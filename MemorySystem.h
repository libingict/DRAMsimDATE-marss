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
#ifndef MEMORYSYSTEM_H
#define MEMORYSYSTEM_H

#include <deque>
#include "SystemConfiguration.h"
#include "Transaction.h"
#include "MemoryController.h"
#include "Rank.h"
#include "IniReader.h"
#include "ClockDomain.h"
#include "Callback.h"

using std::deque;
namespace DRAMSim
{
	//class MemoryController;
	//class Rank;
	class MemorySystem
	{
	public: 
		MemorySystem();
		virtual ~MemorySystem();
		bool addTransaction(Transaction *trans);
		bool addTransaction(bool isWrite, uint64_t addr);
		bool willAcceptTransaction();
		bool willAcceptTransaction(uint64_t addr);
		void update();
		void printStats(bool finalStats);
		void registerCallbacks( TransactionCompleteCB *readDone, TransactionCompleteCB *writeDone,
								void (*reportPower)(double bgpower, double burstpower, double refreshpower, double actprepower));

		void addressMapping(
				uint64_t physicalAddress,
				unsigned &channel,
				unsigned &rank,
				unsigned &bank,
				unsigned &row,
				unsigned &col);

		unsigned findChannelNumber(uint64_t addr);

		//fields
		vector<MemoryController *> memoryControllers;
		vector<vector<Rank *> *> ranks;
		deque<Transaction *> pendingTransactions;

		//function pointers
		TransactionCompleteCB* ReadDataDone;
		TransactionCompleteCB* WriteDataDone;

		//TODO: make this a functor as well?
		static PowerCB ReportPower;

	};
}

#endif
