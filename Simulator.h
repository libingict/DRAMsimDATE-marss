#ifndef SIMULATOR_H_
#define SIMULATOR_H_

#include "SimulatorIO.h"
#include "ClockDomain.h"
#include "MemorySystem.h"
#include "SimpleCache.h"

using BlSim::Cache;
//#define WarmupCycle 1e+5

namespace DRAMSim
{
	class Simulator
	{
	public:
		Simulator(SimulatorIO *simIO) : simIO(simIO),
		                                memorySystem(NULL),
		                                myCache(NULL),
		                                trans(NULL),
		                                evicted_trans(NULL),
		                                pendingTrace(true) {};
		~Simulator();

		void setup();
		void start();
		void update();
		void report(bool finalStats);

		static ClockDomain* clockDomainCPU;
		static ClockDomain* clockDomainDRAM;
		static ClockDomain* clockDomainTREE;
		MemorySystem *memorySystem;
		void setCPUClock(uint64_t cpuClkFreqHz);


	private:
		void setClockRatio(double ratio);

		SimulatorIO *simIO;

		Cache *myCache;
		Transaction *trans;
		Transaction *evicted_trans;


		bool pendingTrace;

#ifdef RETURN_TRANSACTIONS
		TransactionReceiver *transReceiver;
#endif

	};
	Simulator* getMemorySimulator(const string &dev, const string &sys, const string &pwd, const string &trc, unsigned megsOfMemory, std::string *visfilename=NULL) ;
}
#endif /* SIMULATOR_H_ */
