/*********************************************************************************
 *  Copyright (c) 2010-2011, Elliott Cooper-Balis
 *                             Paul Rosenfeld
 *                             Bruce Jacob
 *                             University of Maryland 
 *                             dramninjas [at] gmail [dot] com
 *  All rights reserved.
 *  
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  
 *     * Redistributions of source code must retain the above copyright notice,
 *        this list of conditions and the following disclaimer.
 *  
 *     * Redistributions in binary form must reproduce the above copyright notice,
 *        this list of conditions and the following disclaimer in the documentation
 *        and/or other materials provided with the distribution.
 *  
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************************/

//MemoryController.cpp
//
//Class file for memory controller object
//
#include "MemoryController.h"
#include "MemorySystem.h"
#include "SimulatorIO.h"
#include "Simulator.h"

#define SEQUENTIAL(rank,bank) (rank*NUM_BANKS)+bank
static uint64_t reducedcmd = 0;
static uint64_t completedSET = 0;
static uint64_t eraseSET = 0;
#define threshold 4
namespace DRAMSim {
using std::max;
using std::dec;
using std::hex;
using std::ios;

MemoryController::MemoryController(MemorySystem *parent, vector<Rank *> *ranks,
		unsigned channel) :
		parentMemorySystem(parent), ranks(ranks), bankStates(NUM_RANKS,
				vector<BankState>(NUM_BANKS)), commandQueue(bankStates), poppedBusPacket(
				NULL), totalTransactions(0), refreshRank(0), csvOut(
				SimulatorIO::verifyFile), channelID(channel) {
	//bus related fields
	outgoingCmdPacket = NULL;
	outgoingDataPacket = NULL;
	dataCyclesLeft = 0;
	cmdCyclesLeft = 0;

	//reserve memory for vectors
	transactionQueue.reserve(TRANS_QUEUE_DEPTH);

	powerDown = vector<bool>(NUM_RANKS, false);
	grandTotalBankAccesses = vector<uint64_t>(NUM_RANKS * NUM_BANKS, 0);
	totalReadsPerBank = vector<uint64_t>(NUM_RANKS * NUM_BANKS, 0);
	totalWritesPerBank = vector<uint64_t>(NUM_RANKS * NUM_BANKS, 0);

	fullSETPerBank = vector<uint64_t>(NUM_RANKS * NUM_BANKS, 0);
	flushSETPerBank = vector<uint64_t>(NUM_RANKS * NUM_BANKS, 0);
	EmergePartailSET = vector<uint64_t>(NUM_RANKS * NUM_BANKS, 0);
	setChancePerBank = vector<uint64_t>(NUM_RANKS * NUM_BANKS, 0);
	lockBank = vector<bool>(NUM_RANKS * NUM_BANKS, false);

	locktime = vector<vector<uint64_t> >(NUM_RANKS * NUM_BANKS);
	Idletime = vector<vector<uint64_t> >(NUM_RANKS * NUM_BANKS);
	bingoPerBank = vector<uint64_t>(NUM_RANKS * NUM_BANKS, 0);
	notbingoPerBank = vector<uint64_t>(NUM_RANKS * NUM_BANKS, 0);
	accurancy = vector<float>(NUM_RANKS * NUM_BANKS, 0);
//libing	

	totalReadsPerRank = vector<uint64_t>(NUM_RANKS, 0);
	totalWritesPerRank = vector<uint64_t>(NUM_RANKS, 0);

	writeDataCountdown.reserve(NUM_RANKS);
	writeDataToSend.reserve(NUM_RANKS);
	refreshCountdown.reserve(NUM_RANKS);  //per rank

	//Power related packets
	backgroundEnergy = vector<uint64_t>(NUM_RANKS, 0);
	burstEnergy = vector<uint64_t>(NUM_RANKS, 0);
	actpreEnergy = vector<uint64_t>(NUM_RANKS, 0);
	refreshEnergy = vector<uint64_t>(NUM_RANKS, 0);

	totalEpochLatency = vector<uint64_t>(NUM_RANKS * NUM_BANKS, 0);

	//staggers when each rank is due for a refresh
	for (size_t i = 0; i < NUM_RANKS; i++) {
		refreshCountdown.push_back(
				(int) ((REFRESH_PERIOD / tCK) / NUM_RANKS) * (i + 1));
	}
}

//get a bus packet from either data or cmd bus
void MemoryController::receiveFromBus(BusPacket *bpacket) {
	if (bpacket->busPacketType != BusPacket::DATA) {
		ERROR(
				"== Error - Memory Controller received a non-DATA bus packet from rank");
		bpacket->print();
		exit(0);
	}

	if (DEBUG_BUS) {
		PRINTN(" -- MC Receiving From Data Bus : ");
		bpacket->print();
	}

	//add to return read data queue
	returnTransaction.push_back(
			new Transaction(Transaction::RETURN_DATA, bpacket->physicalAddress,
					bpacket->data, bpacket->len));
	totalReadsPerBank[SEQUENTIAL(bpacket->rank,bpacket->bank)]++;

	// this delete statement saves a mindboggling amount of memory
	delete (bpacket);
}

void MemoryController::updateBankState() {
	//update bank states
	for (size_t i = 0; i < NUM_RANKS; i++) {
		for (size_t j = 0; j < NUM_BANKS; j++) {
			if (bankStates[i][j].stateChangeCountdown > 0) {
				//decrement counters
				bankStates[i][j].stateChangeCountdown--;

				//if counter has reached 0, change state
				if (bankStates[i][j].stateChangeCountdown == 0) {
					switch (bankStates[i][j].lastCommand) {
					//only these commands have an implicit state change
					case BusPacket::WRITE_P:
					case BusPacket::READ_P:
						bankStates[i][j].currentBankState =
								BankState::Precharging;
						bankStates[i][j].lastCommand = BusPacket::PRECHARGE;
						bankStates[i][j].stateChangeCountdown = tRP;
						break;

					case BusPacket::REFRESH:
					case BusPacket::PRECHARGE:
						bankStates[i][j].currentBankState = BankState::Idle;
						break;
					default:
						break;
					}
				}
			}
		}
	}
}

void MemoryController::updateCounter() {
	const uint64_t currentClockCycle = Simulator::clockDomainDRAM->clockcycle;

	//check for outgoing command packets and handle countdowns
	if (outgoingCmdPacket != NULL) {
		cmdCyclesLeft--;
		if (cmdCyclesLeft == 0) //packet is ready to be received by rank
				{
			(*ranks)[outgoingCmdPacket->rank]->receiveFromBus(
					outgoingCmdPacket);
			outgoingCmdPacket = NULL;

		}
	}

	//check for outgoing data packets and handle countdowns
	if (outgoingDataPacket != NULL) {
		dataCyclesLeft--;
		if (dataCyclesLeft == 0) {
		//	outgoingDataPacket->print();
			(*ranks)[outgoingDataPacket->rank]->receiveFromBus(
					outgoingDataPacket);

			//inform upper levels that a write is done
			if(outgoingDataPacket->isSETWRITE==false){	
				if (parentMemorySystem->WriteDataDone != NULL) {
					(*parentMemorySystem->WriteDataDone)(channelID,
							outgoingDataPacket->physicalAddress, currentClockCycle);
				}
			}
			outgoingDataPacket = NULL;
		}
	}


	//if any outstanding write data needs to be sent
	//and the appropriate amount of time has passed (WL)
	//then send data on bus
	//
	//write data held in fifo vector along with countdowns
	if (writeDataCountdown.size() > 0) {
		for (size_t i = 0; i < writeDataCountdown.size(); i++) {
			writeDataCountdown[i]--;
		}

		if (writeDataCountdown[0] == 0) {
			//send to bus and print debug stuff
			if (DEBUG_BUS) {
				PRINTN(" -- MC Issuing On Data Bus    : ");
				writeDataToSend[0]->print();
			}

			// queue up the packet to be sent
			if (outgoingDataPacket != NULL) {
				ERROR("== Error - Data Bus Collision");
				exit(-1);
			}
			outgoingDataPacket = writeDataToSend[0];
			dataCyclesLeft = BL / 2;
			totalTransactions++;
			totalWritesPerBank[SEQUENTIAL(writeDataToSend[0]->rank,writeDataToSend[0]->bank)]++;

			writeDataCountdown.erase(writeDataCountdown.begin());
			writeDataToSend.erase(writeDataToSend.begin());
		}
	}

	//if its time for a refresh issue a refresh
	// else pop from command queue if it's not empty
	if (refreshCountdown[refreshRank] == 0) {
		commandQueue.needRefresh(refreshRank);
		//(*ranks)[refreshRank]->refreshWaiting = true;
		refreshCountdown[refreshRank] = REFRESH_PERIOD / tCK;
		refreshRank++;
		if (refreshRank == NUM_RANKS) {
			refreshRank = 0;
		}
	}
	//if a rank is powered down, make sure we power it up in time for a refresh
	else if (powerDown[refreshRank] && refreshCountdown[refreshRank] <= tXP) {
		//(*ranks)[refreshRank]->refreshWaiting = true;

	}

	//decrement refresh counters
	for (size_t i = 0; i < NUM_RANKS; i++) {
		refreshCountdown[i]--;
	}
}

void MemoryController::updateCmdQueue() {
	const uint64_t currentClockCycle = Simulator::clockDomainDRAM->clockcycle;

	//pass a pointer to a poppedBusPacket
	//function returns true if there is something valid in poppedBusPacket

	if (commandQueue.pop(&poppedBusPacket)) {
		if (SET_IDLE) {
			if (poppedBusPacket->busPacketType == BusPacket::WRITE
					|| poppedBusPacket->busPacketType == BusPacket::WRITE_P) {
				vector<BusPacket*> bsqueue = commandQueue.getCommandQueue(
						poppedBusPacket->rank, poppedBusPacket->bank);
				size_t i;
				bool added = false;
				for (i = 0; i < bsqueue.size(); ++i) {
					if (bsqueue[i]->busPacketType == BusPacket::READ
							|| bsqueue[i]->busPacketType == BusPacket::READ_P) { //has to issue Partial,insert Partial Queue
						Transaction *trans = new Transaction(
								Transaction::DATA_WRITE,
								poppedBusPacket->physicalAddress,
								poppedBusPacket->data, poppedBusPacket->len);
						trans->timeAdded =
								Simulator::clockDomainDRAM->clockcycle;
						addPartialQueue(trans);
						added = true;
						break;
					}
				}
				if (!added) {
					poppedBusPacket->busPacketType == BusPacket::COM_WRITE;
					//poppedBusPacket->busPacketType == BusPacket::SET_WRITE;
					completedSET++;
					if (PSQueue.size() != 0) {
						for (list<Transaction*>::iterator iter =
								PSQueue.begin(); iter != PSQueue.end();
								++iter) {
							Transaction* transaction = *iter;
							if (transaction->address
									== poppedBusPacket->physicalAddress) {
								PSQueue.erase(iter);
								eraseSET++;
								break;
							}
						}
					}
				}
			}
		}
		if (poppedBusPacket->busPacketType == BusPacket::WRITE
				|| poppedBusPacket->busPacketType == BusPacket::SET_WRITE
				|| poppedBusPacket->busPacketType == BusPacket::COM_WRITE
				|| poppedBusPacket->busPacketType == BusPacket::WRITE_P) {
			BusPacket *bpWrite = new BusPacket(BusPacket::DATA,
					poppedBusPacket->rank, poppedBusPacket->bank,
					poppedBusPacket->row, poppedBusPacket->column,
					poppedBusPacket->physicalAddress, poppedBusPacket->data,
					poppedBusPacket->len);
			if(poppedBusPacket->busPacketType==BusPacket::SET_WRITE){
				bpWrite->isSETWRITE=true;			
			}
			writeDataToSend.push_back(bpWrite);

			writeDataCountdown.push_back(WL);
		}

		//
		//update each bank's state based on the command that was just popped out of the command queue
		//
		//for readability's sake
		unsigned rank = poppedBusPacket->rank;
		unsigned bank = poppedBusPacket->bank;
		size_t len = poppedBusPacket->len;

		switch (poppedBusPacket->busPacketType) {
		case BusPacket::READ_P:
		case BusPacket::READ:
			//add energy to account for total
			if (DEBUG_POWER) {
				PRINT(" ++ Adding Read energy to total energy");
			}
			burstEnergy[rank] += (IDD4R - IDD3N) * BL / 2 * len;
			if (poppedBusPacket->busPacketType == BusPacket::READ_P) {
				//Don't bother setting next read or write times because the bank is no longer active
				//bankStates[rank][bank].currentBankState = Idle;
				bankStates[rank][bank].nextActivate = max(
						currentClockCycle + READ_AUTOPRE_DELAY,
						bankStates[rank][bank].nextActivate);
				bankStates[rank][bank].lastCommand = BusPacket::READ_P;
				bankStates[rank][bank].stateChangeCountdown = READ_TO_PRE_DELAY;
			} else if (poppedBusPacket->busPacketType == BusPacket::READ) {
				bankStates[rank][bank].nextPrecharge = max(
						currentClockCycle + READ_TO_PRE_DELAY,
						bankStates[rank][bank].nextPrecharge);
				bankStates[rank][bank].lastCommand = BusPacket::READ;

			}

			for (size_t i = 0; i < NUM_RANKS; i++) {
				for (size_t j = 0; j < NUM_BANKS; j++) {
					if (i != poppedBusPacket->rank) {
						//check to make sure it is active before trying to set (save's time?)
						if (bankStates[i][j].currentBankState
								== BankState::RowActive) {
							bankStates[i][j].nextRead = max(
									currentClockCycle + BL / 2 + tRTRS,
									bankStates[i][j].nextRead);
							bankStates[i][j].nextWrite = max(
									currentClockCycle + READ_TO_WRITE_DELAY,
									bankStates[i][j].nextWrite);
						}
					} else {
						bankStates[i][j].nextRead = max(
								currentClockCycle + max(tCCD, BL / 2),
								bankStates[i][j].nextRead);
						bankStates[i][j].nextWrite = max(
								currentClockCycle + READ_TO_WRITE_DELAY,
								bankStates[i][j].nextWrite);
					}
				}
			}

			if (poppedBusPacket->busPacketType == BusPacket::READ_P) {
				//set read and write to nextActivate so the state table will prevent a read or write
				//  being issued (in cq.isIssuable())before the bank state has been changed because of the
				//  auto-precharge associated with this command
				bankStates[rank][bank].nextRead =
						bankStates[rank][bank].nextActivate;
				bankStates[rank][bank].nextWrite =
						bankStates[rank][bank].nextActivate;

				cmdStat.readpCounter++;
			} else {
				cmdStat.readCounter++;
			}

			break;
		case BusPacket::WRITE_P:
		case BusPacket::WRITE:
		case BusPacket::SET_WRITE:
		case BusPacket::COM_WRITE:
			// if write , check no read following.if has read ,addPartial Queue.


			if (poppedBusPacket->busPacketType == BusPacket::WRITE_P) {
				bankStates[rank][bank].nextActivate = max(
						currentClockCycle + WRITE_AUTOPRE_DELAY,
						bankStates[rank][bank].nextActivate);
				bankStates[rank][bank].lastCommand = BusPacket::WRITE_P;
				bankStates[rank][bank].stateChangeCountdown =
						WRITE_TO_PRE_DELAY;
			} else if (poppedBusPacket->busPacketType == BusPacket::WRITE) {
				bankStates[rank][bank].nextPrecharge = max(
						currentClockCycle + WRITE_TO_PRE_DELAY,
						bankStates[rank][bank].nextPrecharge);
				bankStates[rank][bank].lastCommand = BusPacket::WRITE;
			} else if (poppedBusPacket->busPacketType == BusPacket::SET_WRITE
					|| poppedBusPacket->busPacketType == BusPacket::COM_WRITE) {
				if (SET_CLOSE) {
					bankStates[rank][bank].nextActivate = max(
							currentClockCycle + SET_AUTOPRE_DELAY,
							bankStates[rank][bank].nextActivate);
					bankStates[rank][bank].lastCommand = BusPacket::SET_WRITE;
					bankStates[rank][bank].stateChangeCountdown =
							SET_TO_PRE_DELAY;
				} else {
					bankStates[rank][bank].nextPrecharge = max(
							currentClockCycle + SET_TO_PRE_DELAY,
							bankStates[rank][bank].nextPrecharge);
					bankStates[rank][bank].lastCommand = BusPacket::SET_WRITE;

				}

			}

			//add energy to account for total
			if (DEBUG_POWER) {
				PRINT(" ++ Adding Write energy to total energy");
			}
			burstEnergy[rank] += (IDD4W - IDD3N) * BL / 2 * len;

			for (size_t i = 0; i < NUM_RANKS; i++) {
				for (size_t j = 0; j < NUM_BANKS; j++) {
					if (i != poppedBusPacket->rank) {
						if (bankStates[i][j].currentBankState
								== BankState::RowActive) {
							bankStates[i][j].nextWrite = max(
									currentClockCycle + BL / 2 + tRTRS,
									bankStates[i][j].nextWrite);
							bankStates[i][j].nextRead = max(
									currentClockCycle + WRITE_TO_READ_DELAY_R,
									bankStates[i][j].nextRead);
						}
					} else {
						bankStates[i][j].nextWrite = max(
								currentClockCycle + max(BL / 2, tCCD),
								bankStates[i][j].nextWrite);
						bankStates[i][j].nextRead = max(
								currentClockCycle + WRITE_TO_READ_DELAY_B,
								bankStates[i][j].nextRead);
					}
				}
			}

			//set read and write to nextActivate so the state table will prevent a read or write
			//  being issued (in cq.isIssuable())before the bank state has been changed because of the
			//  auto-precharge associated with this command
			if (poppedBusPacket->busPacketType == BusPacket::WRITE_P) {
				bankStates[rank][bank].nextRead =
						bankStates[rank][bank].nextActivate;
				bankStates[rank][bank].nextWrite =
						bankStates[rank][bank].nextActivate;

				cmdStat.writepCounter++;
			} else {
				cmdStat.writeCounter++;
			}
			break;
		case BusPacket::ACTIVATE:
			//add energy to account for total
			if (DEBUG_POWER) {
				PRINT(
						" ++ Adding Activate and Precharge energy to total energy");
			}
			actpreEnergy[rank] += ((IDD0 * tRC)
					- ((IDD3N * tRAS) + (IDD2N * (tRC - tRAS)))) * len;

			bankStates[rank][bank].currentBankState = BankState::RowActive;
			bankStates[rank][bank].lastCommand = BusPacket::ACTIVATE;
			bankStates[rank][bank].openRowAddress = poppedBusPacket->row;
			bankStates[rank][bank].nextActivate = max(currentClockCycle + tRC,
					bankStates[rank][bank].nextActivate);
			bankStates[rank][bank].nextPrecharge = max(currentClockCycle + tRAS,
					bankStates[rank][bank].nextPrecharge);

			//if we are using posted-CAS, the next column access can be sooner than normal operation

			bankStates[rank][bank].nextRead = max(
					currentClockCycle + (tRCD - AL),
					bankStates[rank][bank].nextRead);
			bankStates[rank][bank].nextWrite = max(
					currentClockCycle + (tRCD - AL),
					bankStates[rank][bank].nextWrite);

			for (size_t i = 0; i < NUM_BANKS; i++) {
				if (i != poppedBusPacket->bank) {
					bankStates[rank][i].nextActivate = max(
							currentClockCycle + tRRD,
							bankStates[rank][i].nextActivate);
				}
			}

			cmdStat.activateCounter++;
			break;
		case BusPacket::PRECHARGE:
			bankStates[rank][bank].currentBankState = BankState::Precharging;
			bankStates[rank][bank].lastCommand = BusPacket::PRECHARGE;
			bankStates[rank][bank].stateChangeCountdown = tRP;
			bankStates[rank][bank].nextActivate = max(currentClockCycle + tRP,
					bankStates[rank][bank].nextActivate);

			cmdStat.prechangeCounter++;
			break;
		case BusPacket::REFRESH:
			//add energy to account for total
			if (DEBUG_POWER) {
				PRINT(" ++ Adding Refresh energy to total energy");
			}
			refreshEnergy[rank] += (IDD5 - IDD3N) * tRFC * NUM_DEVICES;

			for (size_t i = 0; i < NUM_BANKS; i++) {
				bankStates[rank][i].nextActivate = currentClockCycle + tRFC;
				bankStates[rank][i].currentBankState = BankState::Refreshing;
				bankStates[rank][i].lastCommand = BusPacket::REFRESH;
				bankStates[rank][i].stateChangeCountdown = tRFC;
			}

			cmdStat.refreshCounter++;
			break;
		default:
			ERROR(
					"== Error - Popped a command we shouldn't have of type : " << poppedBusPacket->busPacketType)
			;
			exit(0);
		}

		//issue on bus and print debug
		if (DEBUG_BUS) {
			PRINTN(" -- MC Issuing On Command Bus : ");
			poppedBusPacket->print();
		}

		//check for collision on bus
		if (outgoingCmdPacket != NULL) {
			ERROR("== Error - Command Bus Collision");
			exit(-1);
		}
		outgoingCmdPacket = poppedBusPacket;
		cmdCyclesLeft = tCMD;

	}
}

void MemoryController::updateTransQueue() {
	for (size_t i = 0; i < transactionQueue.size(); i++) {
		//pop off top transaction from queue
		//
		//	assuming simple scheduling at the moment
		//	will eventually add policies here
		Transaction *transaction = transactionQueue[i];

		//map address to rank,bank,row,col
		unsigned newChan, newRank, newBank, newRow, newColumn;

		// pass these in as references so they get set by the addressMapping function
		parentMemorySystem->addressMapping(transaction->address, newChan,
				newRank, newBank, newRow, newColumn);

		//if we have room, break up the transaction into the appropriate commands
		//and add them to the command queue
		if (commandQueue.hasRoomFor(2, newRank, newBank)) {
			if (DEBUG_ADDR_MAP) {
				PRINTN(
						"== New Transaction - Mapping Address [0x" << hex << transaction->address << dec << "]");
				if (transaction->transactionType == Transaction::DATA_READ) {
					PRINT(" (Read)");
				} else {
					PRINT(" (Write)");
				}
				PRINT("Channel: " << newChan);
				PRINT("  Rank : " << newRank);
				PRINT("  Bank : " << newBank);
				PRINT("  Row  : " << newRow);
				PRINT(" Column: " << newColumn);

			}

			//now that we know there is room in the command queue, we can remove from the transaction queue
			transactionQueue.erase(transactionQueue.begin() + i);

			//create activate command to the row we just translated
			BusPacket *ACTcommand = new BusPacket(BusPacket::ACTIVATE, newRank,
					newBank, newRow, newColumn, transaction->address,
					transaction->data, transaction->len);

			//create read or write command and enqueue it
			BusPacket::BusPacketType bpType = transaction->getBusPacketType();
			BusPacket *command = new BusPacket(bpType, newRank, newBank, newRow,
					newColumn, transaction->address, transaction->data,
					transaction->len);

			commandQueue.enqueue(ACTcommand);
			commandQueue.enqueue(command);

			// If we have a read, save the transaction so when the data comes back
			// in a bus packet, we can staple it back into a transaction and return it
			if (transaction->transactionType == Transaction::DATA_READ) {
				pendingReadTransactions.push_back(transaction);
			} else {
				if (SET_IDLE) {
					//cout << "add to Partial Queue\n";
					//update the CMD queue, and the time
					scheduleCMD(command, commandQueue);
					//					addPartialQueue(trans);
				}

				// just delete the transaction now that it's a buspacket
				delete transaction;
			}
			/* only allow one transaction to be scheduled per cycle -- this should
			 * be a reasonable assumption considering how much logic would be
			 * required to schedule multiple entries per cycle (parallel data
			 * lines, switching logic, decision logic)
			 */
			break;
		} else // no room, do nothing this cycle
		{
			//PRINT( "== Warning - No room in command queue" << endl;
		}
	}
}

void MemoryController::addPartialQueue(Transaction *trans) { //libing
	if (PSQueue.size() >= MAX_DEPTH) { //todo: what if PQ full
		/*		DEBUG("Partial Queue is FULL!");
		 std::cout << " current cycle is "
		 << Simulator::clockDomainCPU->clockcycle << std::endl;
		 //printPartialQueue();
		 //commandQueue.print();
		 //exit(0);*/
		return;
	}
	trans->timeAdded = Simulator::clockDomainCPU->clockcycle;
	for (list<Transaction*>::iterator iter = PSQueue.begin();
			iter != PSQueue.end(); ++iter) {
		Transaction* transaction = *iter;
		if (transaction->address == trans->address) {
			PSQueue.erase(iter);
			break;
		}
	}
	PSQueue.push_back(trans);
}
void MemoryController::printPartialQueue() {
	for (list<Transaction*>::iterator iter = PSQueue.begin();
			iter != PSQueue.end(); ++iter) {
		Transaction* transaction = *iter;
		uint64_t address = transaction->address;
		uint64_t addtime = transaction->timeAdded;
		uint64_t tracetime = transaction->timeTraced;

		PRINT(
				"T [Write] [0x" << hex << address << "] added time is [" << dec << addtime<< "] trace time is [" << dec << tracetime <<"]");

	}
}

void MemoryController::updatePartialQueue() {
	uint64_t fact;
	uint64_t predict;
	bool flip;
	if (PSQueue.size() == 0) {
		return;
	}

	bool setTimes;
	//map address to rank,bank,row,col
	unsigned newChan, newRank, newBank, newRow, newColumn;
	if (PSQueue.size() == MAX_DEPTH - 1) {
			Transaction *transaction = PSQueue.front();
			parentMemorySystem->addressMapping(transaction->address, newChan,
					newRank, newBank, newRow, newColumn);
			if (commandQueue.hasRoomFor(2, newRank, newBank)) {
			PSQueue.pop_front();

			//create activate command to the row we just translated
			BusPacket *ACTcommand = new BusPacket(BusPacket::ACTIVATE, newRank,
					newBank, newRow, newColumn, transaction->address, NULL,
					transaction->len);

			//create read or write command and enqueue it
			BusPacket *command = new BusPacket(BusPacket::SET_WRITE, newRank,
					newBank, newRow, newColumn, transaction->address, NULL,
					transaction->len);
			commandQueue.insert(command);
			commandQueue.insert(ACTcommand);

			delete (transaction);
			flushSETPerBank[SEQUENTIAL(newRank, newBank)]++;
		}
		return;
	}

/*	for (list<Transaction*>::iterator iter = PSQueue.begin();
			iter != PSQueue.end();) {
		Transaction* transaction = *iter;
		// pass these in as references so they get set by the addressMapping function
		parentMemorySystem->addressMapping(transaction->address, newChan,
				newRank, newBank, newRow, newColumn);
		//DO: detect the idle interval to schedule the Partial SET

		if ((bankStates[newRank][newBank].currentBankState == BankState::Idle
				&& commandQueue.isbankEmpty(newRank, newBank))) {
			uint64_t interval =0 ;
			if (lockBank[SEQUENTIAL(newRank,newBank)] == false) {
				locktime[SEQUENTIAL(newRank,newRank)].push_back(
						Simulator::clockDomainDRAM->clockcycle);
				lockBank[SEQUENTIAL(newRank,newBank)] = true;
				//check the history
			} else {
				vector<uint64_t>::iterator timeiter =
						locktime[SEQUENTIAL(newRank,newBank)].end();
				interval = Simulator::clockDomainDRAM->clockcycle
						- *(--timeiter);
				if (interval >= threshold || commandQueue.isEmpty(newRank)) {
								BusPacket *ACTcommand = new BusPacket(BusPacket::ACTIVATE,
										newRank, newBank, newRow, newColumn,
										transaction->address, NULL, transaction->len);

								//create read or write command and enqueue it
								BusPacket *command = new BusPacket(BusPacket::SET_WRITE,
										newRank, newBank, newRow, newColumn,
										transaction->address, NULL, transaction->len);
								commandQueue.insert(command);
								commandQueue.insert(ACTcommand);

								PSQueue.erase(iter);
								delete (transaction);
								fullSETPerBank[SEQUENTIAL(newRank, newBank)]++;
							}
			}


						if (Idletime[SEQUENTIAL(newRank,newBank)].size() == 0) {
			 setTimes = 0;
			 } else {
			 size_t s = Idletime[SEQUENTIAL(newRank,newBank)].size() - 1;
			 predict = Idletime[SEQUENTIAL(newRank,newBank)][s - 1];
			 fact = Idletime[SEQUENTIAL(newRank,newBank)][s];
			 if ((predict < SET_TO_PRE_DELAY && fact < SET_TO_PRE_DELAY)||(predict >= SET_TO_PRE_DELAY && fact >=SET_TO_PRE_DELAY)){
			 bingoPerBank[SEQUENTIAL(newRank,newBank)]++;
			 }
			 else {
			 notbingoPerBank[SEQUENTIAL(newRank,newBank)]++;
			 }
			 accurancy[SEQUENTIAL(newRank,newBank)] =
			 (float) bingoPerBank[SEQUENTIAL(newRank,newBank)]
			 / (bingoPerBank[SEQUENTIAL(newRank,newBank)]
			 + notbingoPerBank[SEQUENTIAL(newRank,newBank)]);
			 cout << " accuracy is : "
			 << accurancy[SEQUENTIAL(newRank,newBank)] << endl;
			 if (accurancy[SEQUENTIAL(newRank,newBank)] > 0.5) {
			 flip = 1;
			 } else {
			 flip = 0;
			 }
			 vector<uint64_t>::iterator iterofIdle =
			 Idletime[SEQUENTIAL(newRank,newBank)].end();
			 --iterofIdle;
			 if (SET_CLOSE) {
			 if (*(iterofIdle) < SET_AUTOPRE_DELAY) {
			 setTimes = 1;
			 } else {
			 setTimes = 0;
			 }
			 } else {
			 //TODO:if accurate ?
			 if (*(iterofIdle) < SET_TO_PRE_DELAY) {
			 setTimes = 0;
			 } else {
			 setTimes = 1;
			 }
			 if (!flip) {
			 setTimes = !setTimes;
			 }
			 //}
			 }
//			if (setTimes) {
//				PSQueue.pop_front();
//				//create activate command to the row we just translated
//				BusPacket *ACTcommand = new BusPacket(BusPacket::ACTIVATE,
//						newRank, newBank, newRow, newColumn,
//						transaction->address, NULL, transaction->len);
//
//				//create read or write command and enqueue it
//				BusPacket *command = new BusPacket(BusPacket::SET_WRITE,
//						newRank, newBank, newRow, newColumn,
//						transaction->address, NULL, transaction->len);
//
//				commandQueue.enqueue(ACTcommand);
//				commandQueue.enqueue(command);
//
//				delete (transaction);
//				totalSETPerBank[SEQUENTIAL(newRank, newBank)]++;
//			}
			iter++;
		}

	}*/
}
void MemoryController::getIdleInterval() {

	//every bank : lock,empty time,
	//uint64_t idleTime(0);
	for (size_t i = 0; i < NUM_RANKS; i++) {
		for (size_t j = 0; j < NUM_BANKS; j++) {
			if (!SET_IDLE) {
				if (bankStates[i][j].currentBankState == BankState::Idle
						&& commandQueue.isbankEmpty(i, j)) {
					if (lockBank[SEQUENTIAL(i,j)] == false) {
						locktime[SEQUENTIAL(i,j)].push_back(
								Simulator::clockDomainDRAM->clockcycle);
						lockBank[SEQUENTIAL(i,j)] = true;
					}
/*					if (commandQueue.isEmpty(i)){
						setChancePerBank[SEQUENTIAL(i,j)]++;
					}*/
				} else {
					if (lockBank[SEQUENTIAL(i,j)] == true) {
						lockBank[SEQUENTIAL(i,j)] = false;
						vector<uint64_t>::iterator iter =
								locktime[SEQUENTIAL(i,j)].end();
						uint64_t idle = Simulator::clockDomainDRAM->clockcycle
								- *(--iter);
						Idletime[SEQUENTIAL(i,j)].push_back(idle);
						if (commandQueue.isEmpty(i)){
												setChancePerBank[SEQUENTIAL(i,j)]++;
											}
				//		if (idle > SET_TO_PRE_DELAY) {
					//		setChancePerBank[SEQUENTIAL(i,j)]++;
						//}
					}
				}
			}
			if (SET_IDLE) {
				if (lockBank[SEQUENTIAL(i,j)] == true) {
					if (!commandQueue.isbankEmpty(i, j)) {
							lockBank[SEQUENTIAL(i,j)] = false;
							vector<uint64_t>::iterator iter =
									locktime[SEQUENTIAL(i,j)].end();
							uint64_t idle =
									Simulator::clockDomainDRAM->clockcycle
											- *(--iter);
							Idletime[SEQUENTIAL(i,j)].push_back(idle);
							if (idle > SET_TO_PRE_DELAY) {
								setChancePerBank[SEQUENTIAL(i,j)]++;
							}
					}

					/*			switch (bankStates[i][j].currentBankState) {
					 case BankState::Idle: {
					 if (commandQueue.isbankEmpty(i, j)) { //cmd is idle

					 if (lockBank[SEQUENTIAL(i,j)] == false) {
					 locktime[SEQUENTIAL(i,j)].push_back(
					 Simulator::clockDomainDRAM->clockcycle);
					 lockBank[SEQUENTIAL(i,j)] = true;
					 }
					 } else {  //there is no other packet then idle , else busy
					 if (popcmd[0]->busPacketType == BusPacket::ACTIVATE
					 && popcmd[1]->busPacketType
					 == BusPacket::SET_WRITE) { //cmd is idle
					 if (lockBank[SEQUENTIAL(i,j)] == false) {
					 locktime[SEQUENTIAL(i,j)].push_back(
					 Simulator::clockDomainDRAM->clockcycle);
					 lockBank[SEQUENTIAL(i,j)] = true;
					 }
					 } else { //cmd is not idle
					 if (lockBank[SEQUENTIAL(i,j)] == true) {
					 lockBank[SEQUENTIAL(i,j)] = false;
					 vector<uint64_t>::iterator iter =
					 locktime[SEQUENTIAL(i,j)].end();
					 uint64_t idle =
					 Simulator::clockDomainDRAM->clockcycle
					 - *(--iter);
					 Idletime[SEQUENTIAL(i,j)].push_back(idle);
					 if (idle > SET_TO_PRE_DELAY) {
					 setChancePerBank[SEQUENTIAL(i,j)]++;
					 }
					 }
					 }
					 }
					 }
					 break;
					 case BankState::RowActive: {
					 size_t n;		//cmd no other packet then idle else busy
					 for (n = 0; n < popcmd.size(); n++) {
					 if (!(popcmd[n]->busPacketType == BusPacket::SET_WRITE
					 || popcmd[n]->busPacketType
					 == BusPacket::ACTIVATE)) { //busy
					 if (lockBank[SEQUENTIAL(i,j)] == true) {
					 lockBank[SEQUENTIAL(i,j)] = false;
					 vector<uint64_t>::iterator iter =
					 locktime[SEQUENTIAL(i,j)].end();
					 uint64_t idle =
					 Simulator::clockDomainDRAM->clockcycle
					 - *(--iter);
					 Idletime[SEQUENTIAL(i,j)].push_back(idle);
					 if (idle > SET_TO_PRE_DELAY) {
					 setChancePerBank[SEQUENTIAL(i,j)]++;
					 }
					 }
					 break;
					 }
					 }
					 if (n != 0 && (n == popcmd.size())) { //cmd is checked and no other packet, then idle
					 if (lockBank[SEQUENTIAL(i,j)] == false) {
					 locktime[SEQUENTIAL(i,j)].push_back(
					 Simulator::clockDomainDRAM->clockcycle);
					 lockBank[SEQUENTIAL(i,j)] = true;
					 }
					 }
					 }
					 break;
					 default: {
					 if (lockBank[SEQUENTIAL(i,j)] == true) {
					 lockBank[SEQUENTIAL(i,j)] = false;
					 vector<uint64_t>::iterator iter =
					 locktime[SEQUENTIAL(i,j)].end();
					 uint64_t idle = Simulator::clockDomainDRAM->clockcycle
					 - *(--iter);
					 Idletime[SEQUENTIAL(i,j)].push_back(idle);
					 if (idle > SET_TO_PRE_DELAY) {
					 setChancePerBank[SEQUENTIAL(i,j)]++;
					 }
					 }
					 }*/
				}
			}
		}
	}
}

void MemoryController::scheduleCMD(BusPacket *busPacket,
		CommandQueue &cmdQueue) {
	unsigned rank = busPacket->rank;
	unsigned bank = busPacket->bank;
	unsigned address = busPacket->physicalAddress;
	vector<BusPacket*> queue = cmdQueue.getCommandQueue(rank, bank);
	vector<BusPacket*>::iterator enditer = queue.end();
	/*	if (*busPacket != *(enditer - 1)) {
	 return;
	 }*/
	if (queuingStructure == PerRankPerBank) {
		for (vector<BusPacket*>::iterator iter = queue.begin();
				iter < queue.end(); ++iter) {
			BusPacket * bpacket = *iter;
			if (bpacket->physicalAddress == address) {	//same address
				if (bpacket->busPacketType == BusPacket::WRITE
						|| bpacket->busPacketType == BusPacket::WRITE_P
						|| bpacket->busPacketType == BusPacket::COM_WRITE
						|| bpacket->busPacketType == BusPacket::SET_WRITE) {
					if (busPacket->busPacketType == BusPacket::READ
							|| busPacket->busPacketType == BusPacket::READ_P) {	//read, return the data obtained from write
						busPacket->busPacketType = BusPacket::DATA;
						queue.erase(queue.end(), queue.end() - 1);
						reducedcmd++;
						receiveFromBus(busPacket);
						//then return the read transaction
					} else if (busPacket->busPacketType == BusPacket::WRITE
							|| busPacket->busPacketType == BusPacket::WRITE_P) {//write, update the newest
						if (iter != queue.begin()) {
							queue.erase(iter - 1);
						}
						queue.erase(iter);
						reducedcmd++;
					}
				}
			}
			break;
		}
	} else {
		//dont care
		return;
	}
}

void MemoryController::issuePartialSET() {

	const uint64_t currentClockCycle = Simulator::clockDomainDRAM->clockcycle;
	if (PSQueue.size() == 0) {
		return;
	}
	for (list<Transaction*>::iterator iter = PSQueue.begin();
			iter != PSQueue.end(); ++iter) {
		Transaction* trans = *iter;
		// ʱ�䳬����ֵ����Ҫpartial set
		if ((currentClockCycle - trans->timeAdded) >= RETAIN_TIME) {
			unsigned newChan, newRank, newBank, newRow, newColumn;

			// pass these in as references so they get set by the addressMapping function
			parentMemorySystem->addressMapping(trans->address, newChan, newRank,
					newBank, newRow, newColumn);
			BusPacket * queue = new BusPacket(BusPacket::WRITE, newRank,
					newBank, newRow, newColumn, trans->address, NULL, LEN_DEF);
			BusPacket * actqueue = new BusPacket(BusPacket::ACTIVATE, newRank,
					newBank, newRow, newColumn, trans->address, NULL, LEN_DEF);
			//				commandQueue.getCommandQueue(newRank, newBank);
			if (commandQueue.hasRoomFor(2, newRank, newBank)) {
				commandQueue.insert(queue);
				commandQueue.insert(actqueue);
				EmergePartailSET[SEQUENTIAL(newRank,newBank)]++;
			}
		} else {
			break;
		}
	}
}
void MemoryController::updatePower() {
	const uint64_t currentClockCycle = Simulator::clockDomainDRAM->clockcycle;

	//calculate power
	//  this is done on a per-rank basis, since power characterization is done per device (not per bank)
	for (size_t i = 0; i < NUM_RANKS; i++) {
		if (USE_LOW_POWER) {
			//if there are no commands in the queue and that particular rank is not waiting for a refresh...
			if (commandQueue.isEmpty(i) && !(*ranks)[i]->refreshWaiting) {
				//check to make sure all banks are idle
				bool allIdle = true;
				for (size_t j = 0; j < NUM_BANKS; j++) {
					if (bankStates[i][j].currentBankState != BankState::Idle) {
						allIdle = false;
						break;
					}
				}

				//if they ARE all idle, put in power down mode and set appropriate fields
				if (allIdle) {

					powerDown[i] = true;
					(*ranks)[i]->powerDown();
					for (size_t j = 0; j < NUM_BANKS; j++) {
						bankStates[i][j].currentBankState =
								BankState::PowerDown;
						bankStates[i][j].nextPowerUp = currentClockCycle + tCKE;
					}

				}
			}
			//if there IS something in the queue or there IS a refresh waiting (and we can power up), do it
			else if (currentClockCycle >= bankStates[i][0].nextPowerUp
					&& powerDown[i]) //use 0 since theyre all the same
					{
				powerDown[i] = false;
				(*ranks)[i]->powerUp();
				for (size_t j = 0; j < NUM_BANKS; j++) {
					bankStates[i][j].currentBankState = BankState::Idle;
					bankStates[i][j].nextActivate = currentClockCycle + tXP;
				}
			}
		}

		//check for open bank
		bool bankOpen = false;
		for (size_t j = 0; j < NUM_BANKS; j++) {
			if (bankStates[i][j].currentBankState == BankState::Refreshing
					|| bankStates[i][j].currentBankState
							== BankState::RowActive) {
				bankOpen = true;
				break;
			}
		}

		//background power is dependent on whether or not a bank is open or not
		if (bankOpen) {
			if (DEBUG_POWER) {
				PRINT(" ++ Adding IDD3N to total energy [from rank "<< i <<"]");
			}
			backgroundEnergy[i] += IDD3N * NUM_DEVICES;
		} else {
			//if we're in power-down mode, use the correct current
			if (powerDown[i]) {
				if (DEBUG_POWER) {
					PRINT(
							" ++ Adding IDD2P to total energy [from rank " << i << "]");
				}
				backgroundEnergy[i] += IDD2P * NUM_DEVICES;
			} else {
				if (DEBUG_POWER) {
					PRINT(
							" ++ Adding IDD2N to total energy [from rank " << i << "]");
				}
				backgroundEnergy[i] += IDD2N * NUM_DEVICES;
			}
		}
	}
}

void MemoryController::updateReturnTrans() {
	//check for outstanding data to return to the CPU
	if (returnTransaction.size() > 0) {
		if (DEBUG_BUS) {
			PRINTN(" -- MC Issuing to CPU bus : ");
			returnTransaction[0]->print();
		}
		totalTransactions++;

		bool foundMatch = false;
		//find the pending read transaction to calculate latency
		for (size_t i = 0; i < pendingReadTransactions.size(); i++) {
			if (pendingReadTransactions[i]->address
					== returnTransaction[0]->address) {
				//if(currentClockCycle - pendingReadTransactions[i]->timeAdded > 2000)
				//	{
				//		pendingReadTransactions[i]->print();
				//		exit(0);
				//	}

				unsigned chan, rank, bank, row, col;
				parentMemorySystem->addressMapping(
						returnTransaction[0]->address, chan, rank, bank, row,
						col);
				insertHistogram(
						Simulator::clockDomainCPU->clockcycle
								- pendingReadTransactions[i]->timeAdded, rank,
						bank);
				//return latency
				if (DEBUG_ADDR_MAP)				// //added by libing 2013-4-23
				{
					if (pendingReadTransactions[i]->transactionType
							== Transaction::DATA_READ)					//
							{
						PRINT(
								"Read access Address [0x" << hex << pendingReadTransactions[i]->address << dec << "]");
					} else {
						PRINT(
								"Write access Address [0x" << hex << pendingReadTransactions[i]->address << dec << "]");
					}
					PRINT(
							"  Bank : " << bank <<"  issue  time: " << pendingReadTransactions[i]->timeAdded << " return time: " << Simulator::clockDomainDRAM->clockcycle); //added by libing 2013-4-23
				}
				if (parentMemorySystem->ReadDataDone != NULL) {
					(*parentMemorySystem->ReadDataDone)(channelID,
							pendingReadTransactions[i]->address,
							Simulator::clockDomainDRAM->clockcycle);
				}

				delete pendingReadTransactions[i];
				pendingReadTransactions.erase(
						pendingReadTransactions.begin() + i);
				foundMatch = true;
				break;
			}
		}
		if (!foundMatch) {
			ERROR(
					"Can't find a matching transaction for 0x"<<hex<<returnTransaction[0]->address<<dec);
			abort();
		}
		delete returnTransaction[0];
		returnTransaction.erase(returnTransaction.begin());
	}
}

void MemoryController::updatePrint() {
	const uint64_t currentClockCycle = Simulator::clockDomainDRAM->clockcycle;

	//
	//print debug
	//
	if (DEBUG_TRANS_Q) {
		//	PRINT("== Printing transaction queue");
		for (size_t i = 0; i < transactionQueue.size(); i++) {
			PRINT("== Printing transaction queue");
			PRINTN("  " << i << "]");
			transactionQueue[i]->print();
		}
	}

	if (DEBUG_BANKSTATE) {
		//TODO: move this to BankState.cpp
		PRINT("== Printing bank states (According to MC)");
		for (size_t i = 0; i < NUM_RANKS; i++) {
			for (size_t j = 0; j < NUM_BANKS; j++) {
				if (bankStates[i][j].currentBankState == BankState::RowActive) {
					PRINTN("[" << bankStates[i][j].openRowAddress << "] ");
				} else if (bankStates[i][j].currentBankState
						== BankState::Idle) {
					PRINTN("[idle] ");
				} else if (bankStates[i][j].currentBankState
						== BankState::Precharging) {
					PRINTN("[pre] ");
				} else if (bankStates[i][j].currentBankState
						== BankState::Refreshing) {
					PRINTN("[ref] ");
				} else if (bankStates[i][j].currentBankState
						== BankState::PowerDown) {
					PRINTN("[lowp] ");
				}
			}
			PRINT(""); // effectively just cout<<endl;
		}
	}

	if (DEBUG_CMD_Q) {
		commandQueue.print();
	}

	//print stats if we're at the end of an epoch
		if (EPOCH_LENGTH != 0 && currentClockCycle != 0
	 && currentClockCycle % EPOCH_LENGTH == 0)
{

		this->printStats();

		/*
		 cmdStat.readCounter=0;
		 cmdStat.readpCounter=0;
		 cmdStat.writeCounter=0;
		 cmdStat.writepCounter=0;
		 cmdStat.activateCounter=0;
		 cmdStat.prechangeCounter=0;
		 cmdStat.readCounter=0;
		 */

		totalTransactions = 0;
		for (size_t i = 0; i < NUM_RANKS; i++) {
			for (size_t j = 0; j < NUM_BANKS; j++) {
				//XXX: this means the bank list won't be printed for partial epochs
				totalReadsPerBank[SEQUENTIAL(i,j)] = 0;
				totalWritesPerBank[SEQUENTIAL(i,j)] = 0;
				totalEpochLatency[SEQUENTIAL(i,j)] = 0;
				fullSETPerBank[SEQUENTIAL(i,j)] = 0;

			}

			burstEnergy[i] = 0;
			actpreEnergy[i] = 0;
			refreshEnergy[i] = 0;
			backgroundEnergy[i] = 0;
			totalReadsPerRank[i] = 0;
			totalWritesPerRank[i] = 0;
		}
	}
}

void MemoryController::update() {

	//PRINT(" ------------------------- [" << currentClockCycle << "] -------------------------");

	updateBankState();

	updateCounter();

	updateCmdQueue();

	updateTransQueue();

	updateReturnTrans();

	updatePower();
	if (SET_IDLE) {

		updatePartialQueue();

		issuePartialSET();
	}
	else {
		getIdleInterval();
	}

	updatePrint();

}

//allows outside source to make request of memory system
bool MemoryController::addTransaction(Transaction *trans) {
	if (transactionQueue.size() < TRANS_QUEUE_DEPTH) {
		trans->timeAdded = Simulator::clockDomainCPU->clockcycle;
		transactionQueue.push_back(trans);
		return true;
	} else {
		return false;
	}
}
bool MemoryController::WillAcceptTransaction()
	{
	 return transactionQueue.size() < TRANS_QUEUE_DEPTH;
	}
//prints statistics at the end of an epoch or  simulation
void MemoryController::printStats(bool finalStats) {
	const uint64_t currentClockCycle = Simulator::clockDomainDRAM->clockcycle;

	//if we are not at the end of the epoch, make sure to adjust for the actual number of cycles elapsed

	uint64_t cyclesElapsed;
	if (EPOCH_LENGTH == 0) {
//		cyclesElapsed = currentClockCycle - WarmupCycle;
				cyclesElapsed = currentClockCycle;
	} else if (currentClockCycle % EPOCH_LENGTH == 0) {
		cyclesElapsed = EPOCH_LENGTH;
	} else {
		cyclesElapsed = currentClockCycle % EPOCH_LENGTH;
	}

	unsigned bytesPerTransaction = (JEDEC_DATA_BUS_BITS * BL) / 8;
	uint64_t totalBytesTransferred = totalTransactions * bytesPerTransaction;
	double secondsThisEpoch = (double) cyclesElapsed * tCK * 1E-9;

	// only per rank
	vector<double> backgroundPower = vector<double>(NUM_RANKS, 0.0);
	vector<double> burstPower = vector<double>(NUM_RANKS, 0.0);
	vector<double> refreshPower = vector<double>(NUM_RANKS, 0.0);
	vector<double> actprePower = vector<double>(NUM_RANKS, 0.0);
	vector<double> averagePower = vector<double>(NUM_RANKS, 0.0);
	vector<uint64_t> totalSETPerRank = vector<uint64_t>(NUM_RANKS, 0);
	// per bank variables
	vector<double> averageLatency = vector<double>(NUM_RANKS * NUM_BANKS, 0.0);
	vector<double> bandwidth = vector<double>(NUM_RANKS * NUM_BANKS, 0.0);

	double totalBandwidth = 0.0;
	for (size_t i = 0; i < NUM_RANKS; i++) {
		for (size_t j = 0; j < NUM_BANKS; j++) {
			bandwidth[SEQUENTIAL(i,j)] =
					(((double) (totalReadsPerBank[SEQUENTIAL(i,j)]
							+ totalWritesPerBank[SEQUENTIAL(i,j)])
							* (double) bytesPerTransaction)
							/ (1024.0 * 1024.0 * 1024.0)) / secondsThisEpoch;
			averageLatency[SEQUENTIAL(i,j)] =
					((float) totalEpochLatency[SEQUENTIAL(i,j)]
							/ (float) (totalReadsPerBank[SEQUENTIAL(i,j)]))
							* tCK;
			totalBandwidth += bandwidth[SEQUENTIAL(i,j)];
			totalReadsPerRank[i] += totalReadsPerBank[SEQUENTIAL(i,j)];
			totalWritesPerRank[i] += totalWritesPerBank[SEQUENTIAL(i,j)];
			grandTotalBankAccesses[SEQUENTIAL(i,j)] +=
					totalReadsPerBank[SEQUENTIAL(i,j)]
							+ totalWritesPerBank[SEQUENTIAL(i,j)];
		}
	}
	double tAveLatency;
	for (size_t i = 0; i < NUM_RANKS; i++) {
		for (size_t j = 0; j < NUM_BANKS; j++) {
			tAveLatency += averageLatency[SEQUENTIAL(i,j)];
		}
	}
#ifdef LOG_OUTPUT
	SimulatorIO::logFile.precision(3);
	SimulatorIO::logFile.setf(ios::fixed,ios::floatfield);
#else
	cout.precision(3);
	cout.setf(ios::fixed, ios::floatfield);
#endif

	PRINT(" =======================================================");
	PRINT(
			" ============== Printing Statistics [id:"<<channelID<<"]==============");
	PRINTN("  == Total Return Transactions : " << totalTransactions);
	PRINT(
			" ("<<totalBytesTransferred <<" bytes) aggregate average bandwidth "<<totalBandwidth<<"GB/s");
	PRINT(
			"  == Pending Transactions : "<<pendingReadTransactions.size()<<" ("<<currentClockCycle<<")==  CycleElapse: " << cyclesElapsed);
	if (SET_IDLE) {
		PRINT("== Partial Queue size is :"<< PSQueue.size()<<" == ");
	}
	PRINT(
			"      -Total    Average    Latency  :\t\t\t"<< tAveLatency/(NUM_RANKS*NUM_BANKS) <<" ns");

	// only the first memory channel should print the timestamp
	if (VIS_FILE_OUTPUT && channelID == 0) {
		csvOut << "ms" << currentClockCycle * tCK * 1E-6;
	}

	double totalAggregateBandwidth = 0.0;

	for (size_t r = 0; r < NUM_RANKS; r++) {

		PRINT("    -Rank   "<<r<<" : ");
		PRINTN("        -Reads  : " << totalReadsPerRank[r]);
		PRINT(" ("<<totalReadsPerRank[r] * bytesPerTransaction<<" bytes)");
		PRINTN("        -Writes : " << totalWritesPerRank[r]);
		PRINT(" ("<<totalWritesPerRank[r] * bytesPerTransaction<<" bytes)");
	}

		/*	 for (size_t j = 0; j < NUM_BANKS; j++) {
		 PRINT(
		 "      -Bandwidth / Latency  (Bank " <<j<<"): " <<bandwidth[SEQUENTIAL(r,j)] << " GB/s\t" <<averageLatency[SEQUENTIAL(r,j)] << " ns");

		 }

		 double totalRW = cmdStat.readCounter + cmdStat.readpCounter
		 + cmdStat.writeCounter + cmdStat.writepCounter;
		 PRINT(
		 "      -Workload Character[(clock*tck)/(Read+Write)] = \t"<< (cyclesElapsed*tCK)/totalRW <<" ns");

		 // factor of 1000 at the end is to account for the fact that totalEnergy is accumulated in mJ since IDD values are given in mA
		 backgroundPower[r] = ((double) backgroundEnergy[r]
		 / (double) (cyclesElapsed)) * Vdd / 1000.0;
		 burstPower[r] = ((double) burstEnergy[r] / (double) (cyclesElapsed))
		 * Vdd / 1000.0;
		 refreshPower[r] = ((double) refreshEnergy[r] / (double) (cyclesElapsed))
		 * Vdd / 1000.0;
		 actprePower[r] = ((double) actpreEnergy[r] / (double) (cyclesElapsed))
		 * Vdd / 1000.0;
		 averagePower[r] = ((backgroundEnergy[r] + burstEnergy[r]
		 + refreshEnergy[r] + actpreEnergy[r]) / (double) cyclesElapsed)
		 * Vdd / 1000.0;

		 if (MemorySystem::ReportPower != NULL) {
		 (*MemorySystem::ReportPower)(backgroundPower[r], burstPower[r],
		 refreshPower[r], actprePower[r]);
		 }


		 PRINT("  == Power Data for Rank           " << r);
		 PRINT("      -Average Power (watts)     : " << averagePower[r]);
		 PRINT("      -Background    (watts)     : " << backgroundPower[r]);
		 PRINT("      -Act/Pre       (watts)     : " << actprePower[r]);
		 PRINT("      -Burst         (watts)     : " << burstPower[r]);
		 PRINT("      -Refresh       (watts)     : " << refreshPower[r]);


		 if (VIS_FILE_OUTPUT) {
		 // write the vis file output
		 csvOut << CSVWriter::IndexedName("Background_Power", channelID, r)
		 << backgroundPower[r];
		 csvOut << CSVWriter::IndexedName("ACT_PRE_Power", channelID, r)
		 << actprePower[r];
		 csvOut << CSVWriter::IndexedName("Burst_Power", channelID, r)
		 << burstPower[r];
		 csvOut << CSVWriter::IndexedName("Refresh_Power", channelID, r)
		 << refreshPower[r];
		 double totalRankBandwidth = 0.0;
		 for (size_t b = 0; b < NUM_BANKS; b++) {
		 csvOut << CSVWriter::IndexedName("Bandwidth", channelID, r, b)
		 << bandwidth[SEQUENTIAL(r,b)];
		 totalRankBandwidth += bandwidth[SEQUENTIAL(r,b)];
		 totalAggregateBandwidth += bandwidth[SEQUENTIAL(r,b)];
		 csvOut
		 << CSVWriter::IndexedName("Average_Latency", channelID,
		 r, b) << averageLatency[SEQUENTIAL(r,b)];
		 }
		 csvOut
		 << CSVWriter::IndexedName("Rank_Aggregate_Bandwidth",
		 channelID, r) << totalRankBandwidth;
		 csvOut
		 << CSVWriter::IndexedName("Rank_Average_Bandwidth",
		 channelID, r) << totalRankBandwidth / NUM_RANKS;
		 }
		 }*/

		if (VIS_FILE_OUTPUT) {
			csvOut << CSVWriter::IndexedName("Aggregate_Bandwidth", channelID)
					<< totalAggregateBandwidth;
			csvOut << CSVWriter::IndexedName("Average_Bandwidth", channelID)
					<< totalAggregateBandwidth / (NUM_RANKS * NUM_BANKS);
			csvOut.finalize();
		}

		// only print the latency histogram at the end of the simulation since it clogs the output too much to print every epoch
		if (finalStats) {
			PRINT(" =======================================================");
			PRINT("  ==  Final Statistics ==");

			/*		PRINT(" ---  Latency list ("<<latencies.size()<<")");
			 PRINT("    [lat] : #");
			 if (VIS_FILE_OUTPUT) {
			 SimulatorIO::visFile << "!!HISTOGRAM_DATA" << endl;
			 }

			 map<unsigned, unsigned>::iterator it; //
			 for (it = latencies.begin(); it != latencies.end(); it++) {
			 PRINT(
			 "    ["<< it->first <<"-"<<it->first+(HISTOGRAM_BIN_SIZE-1)<<"] : "<< it->second);
			 if (VIS_FILE_OUTPUT) {
			 SimulatorIO::visFile << it->first << "=" << it->second << endl;
			 }
			 }*/
			//IdlePredictStatistic();
/*			if (!SET_IDLE) {
			size_t shortidle=0;
			size_t longidle=0;
			PRINT(
					"	the SET_TO_PRE_DELAY is "<< SET_TO_PRE_DELAY<<"  Rank 0, Bank 0 :");

			for (size_t d = 0; d != Idletime[SEQUENTIAL(0,0)].size(); ++d) {
				if (Idletime[SEQUENTIAL(0,0)][d] > SET_TO_PRE_DELAY) {
					longidle++;
					PRINT(
							"	the "<< d<<" idle cycle is long : "<<Idletime[SEQUENTIAL(0,0)][d]<<" is "<< Idletime[SEQUENTIAL(0,0)][d]/SET_TO_PRE_DELAY<<" X SET time");
				}
				else{
					shortidle++;
				}
			}

			PRINT("	long idle has  "<<longidle <<" times, short idle has :"<<shortidle<<" times.");
			}*/
			PRINT(" --- Grand Total Bank usage list");
			for (size_t i = 0; i < NUM_RANKS; i++) {

				PRINT("  Rank "<<i<<":");

					for (size_t j = 0; j < NUM_BANKS; j++) {
						if (!SET_IDLE) {
						PRINT(
								"	bank"<<j<<": "<<grandTotalBankAccesses[SEQUENTIAL(i,j)]);
						PRINT(
								"	setChance  bank "<<j<<": "<<setChancePerBank[SEQUENTIAL(i,j)]);
						PRINT(
								"	total Idle times bank " <<j<<" is "<<Idletime[SEQUENTIAL(i,j)].size());
					}
						else{
							totalSETPerRank[i] +=flushSETPerBank[SEQUENTIAL(i,j)];
							PRINT("	total SET write from Partial Queue ( bank "<<j<<" ):"<< fullSETPerBank[SEQUENTIAL(i, j)]);
							PRINT("	SET because queue is full for  ( bank "<<j<<" ):"<< flushSETPerBank[SEQUENTIAL(i, j)]);
							PRINT("	SET because retention time for  ( bank "<<j<<" ):"<<EmergePartailSET[SEQUENTIAL(i, j)]);
							//PRINT(
						//		"   The accuracy of prediction for bank  "<< s <<" is: "<<accurancy[SEQUENTIAL(i,s)]);
						}


				}
					if(SET_IDLE){
						PRINTN("	- flush SETs per rank: " << totalSETPerRank[i]);
					}

			}
			if (SET_IDLE) {
				PRINT(
						"   Reduece cmd by Schedule CMDqueue is :" << reducedcmd);
				PRINT("    Completed SET one time :" << completedSET);
				PRINT("The entry erased from queue is " << eraseSET);
			}
			/*	PRINT(" --- DDR DRAM Command Statistics");
			 PRINT("    READ:" << cmdStat.readCounter);
			 PRINT("    READ_P:" << cmdStat.readpCounter);
			 PRINT("    WRITE:" << cmdStat.writeCounter);
			 PRINT("    WRITE_P:" << cmdStat.writepCounter);
			 PRINT("    ACTIVATE:" << cmdStat.activateCounter);
			 PRINT("    PRECHANGE:" << cmdStat.prechangeCounter);
			 PRINT("    REFRESH:" << cmdStat.refreshCounter);*/

#ifdef LOG_OUTPUT
			SimulatorIO::logFile.flush();
#endif
		}

}

/*void MemoryController::IdlePredictStatistic() {
 vector<uint64_t> bingoPerBank;
 bingoPerBank = vector<uint64_t>(NUM_RANKS * NUM_BANKS, 0);
 vector<float> accurancy;
 accurancy = vector<float>(NUM_RANKS * NUM_BANKS, 0);
 uint64_t history;
 uint64_t run_msg;
 for (size_t p = 0; p < NUM_RANKS; p++) {
 for (size_t q = 0; q < NUM_BANKS; q++) {
 for (size_t d = 1; d < Idletime[SEQUENTIAL(p,q)].size(); ++d) {
 if (SET_CLOSE) {
 history = Idletime[SEQUENTIAL(p,q)][d - 1]
 / SET_AUTOPRE_DELAY;
 run_msg = Idletime[SEQUENTIAL(p,q)][d] / SET_AUTOPRE_DELAY;
 } else {
 history = Idletime[SEQUENTIAL(p,q)][d - 1];
 run_msg = Idletime[SEQUENTIAL(p,q)][d];
 }
 if ((run_msg>=SET_TO_PRE_DELAY )&&( history >= SET_TO_PRE_DELAY)) {
 bingoPerBank[SEQUENTIAL(p,q)]++;
 }
 }
 PRINT(
 "The bingo Idle predict of bank " <<q<<" is:" <<bingoPerBank[SEQUENTIAL(p,q)]);
 PRINT(
 "The Idle times of bank "<< q <<" is: "<<Idletime[SEQUENTIAL(p,q)].size());

 accurancy[SEQUENTIAL(p,q)] = (float) bingoPerBank[SEQUENTIAL(p,q)]
 / Idletime[SEQUENTIAL(p,q)].size();
 PRINT(
 "The accurancy of prediction for bank  "<< q <<" is: "<<accurancy[SEQUENTIAL(p,q)]);
 }
 }
 }*/

MemoryController::~MemoryController() {
	//ERROR("MEMORY CONTROLLER DESTRUCTOR");
	//abort();

	for (size_t i = 0; i < pendingReadTransactions.size(); i++) {
		delete pendingReadTransactions[i];
	}
	for (size_t i = 0; i < returnTransaction.size(); i++) {
		delete returnTransaction[i];
	}
	/*		list<Transaction*>::iterator iter;
	 iter a = PSQueue.begin();
	 iter b = PSQueue.end();
	 */
	for (list<Transaction*>::iterator iter = PSQueue.begin();
			iter != PSQueue.end(); ++iter) {
		delete *iter;
	}
}

//inserts a latency into the latency histogram
void MemoryController::insertHistogram(unsigned latencyValue, unsigned rank,
		unsigned bank) {
	totalEpochLatency[SEQUENTIAL(rank,bank)] += latencyValue;
	//poor man's way to bin things.
	latencies[(latencyValue / HISTOGRAM_BIN_SIZE) * HISTOGRAM_BIN_SIZE]++;
}

//libing

}// end of namespace DRAMSim

