#include <iostream>
using namespace std;

#include <string.h>
#include <math.h> 
#include <stdlib.h>
#include <stdio.h>

#include <fstream> // for ifstream
#include <iomanip> // std::setprecision

#include "Cache.h"

#define MAX_TRACE_FILE_SIZE 10000
long glSeqNumCtr = 0;
long glRetireCtr = 0;

// Instruction bundle will have "Width" number of instructions
struct instruction_bundle
{
	int program_counter, operation_type, dest_reg, src1_reg, src2_reg;
};

// Stores the instruction in string format and sequence number
// Fetch -| DE | - Decode
struct DE
{
	bool bValid;
	long lSeqNum;
	int iDuration;
	long lBeginCycle, lFinishCycle; // Begin cycle of Fetch stage
	char strInstruction[32];
};

// Stores the instruction and sequence number
struct RN
{
	bool bValid;
	long lSeqNum;
	int iDuration;
	long lBeginCycle, lFinishCycle;
	int program_counter, operation_type, dest_reg, src1_reg, src2_reg;
};

// Stores renamed register
struct RR
{
	bool bValid;
	long lSeqNum;
	int iDuration;
	long lBeginCycle, lFinishCycle;
	int program_counter, operation_type, rn_dest_reg, rn_src1_reg, rn_src2_reg;
	bool bSrc1Rdy, bSrc2Rdy; // Found in Arf
};

// Stores registers to dispatched
struct DI
{
	bool bValid;
	long lSeqNum;
	int iDuration;
	long lBeginCycle, lFinishCycle;
	int program_counter, operation_type, rn_dest_reg, rn_src1_reg, rn_src2_reg;
	bool bSrc1Rdy, bSrc2Rdy;
};

struct IQ
{
	bool bValid;
	long lSeqNum;
	int iDuration;
	long lBeginCycle, lFinishCycle;
	int program_counter, iDest, iSrc1, iSrc2, operation_type;
	bool bSrc1Rdy, bSrc2Rdy;
};

struct EX
{
	bool bValid;
	long lSeqNum;
	int iDuration;
	long program_counter, lBeginCycle, lFinishCycle;
	int iDest, iSrc1, iSrc2, operation_type, iExecuteLatency;
};

struct WB
{
	bool bValid;
	long lSeqNum;
	int iDuration;
	long program_counter, lBeginCycle, lFinishCycle;
	int iDest, iSrc1, iSrc2, operation_type;
};

//struct RT
//{
//	bool bValid;
//	long lSeqNum;
//	int iDuration;
//	long program_counter, lBeginCycle, lFinishCycle;
//	int iDest, iSrc1, iSrc2, operation_type;
//};

// Holds all the fields needed in the validation file
struct SimOutput
{
	long lSeqNum;
	DE de;
	RN rn;
	RR rr;
	DI di;
	IQ iq;
	EX ex;
	WB wb;
	//RT rt;
	long lWritebackBeginCycle;
	int iWriteBackDuration;
	long lRetireBeginCycle;
	int iRetireDuration;
};

struct rob_entry
{
	long lValue;
	int iDestRegister;
	bool bReady;
	unsigned long ulProgramCtr;
	long lSeqNum; // ACS
};

struct rmt_entry
{
	bool bValid;
	long lRobTag;
};

void printCommandLine(int argc, char* argv[])
{
	cout << "# === Simulator Command =========\n";
	cout << "# ./sim_ds ";
	for (int i = 1; i < argc; i++)
	{
		cout << argv[i] << " ";
	}
	cout << endl;
	cout << "# === Processor Configuration ===\n";
	cout << "# ROB_SIZE 	= " << argv[1] << endl;
	cout << "# IQ_SIZE  	= " << argv[2] << endl;
	cout << "# WIDTH    	= " << argv[3] << endl;
	cout << "# CACHE_SIZE 	= " << argv[4] << endl;
	cout << "# PREFETCHING	= " << argv[5] << endl;
}

class InstructionScheduler
{
public:
	// Command-line input
	long rob_size, instr_queue_size, width, cache_size, prefetch;
	FILE * pFile;

	long lSimulatorCycle;
	bool bTraceDepleted;

	long arf[67];
	rmt_entry rmt[67];
	rob_entry* rob;
	long lRobHead;
	long lRobTail;
	long lVacancyInRob;

	DE* de;
	RN* rn;
	RR* rr;
	DI* di;
	IQ* iq;
	EX* ex;
	WB* wb;
	
	SimOutput* simOutput;

	// Instruction cache
	Cache* ICache;
	int timer;
	bool bFetchInProgress;
	int iFetchedInstr;

	// Prefetch
	bool bPrefetchInProgress;
	int prefetchTimer;
	bool bStall;
	int address;
public:
	bool advance_cycle();
	
	InstructionScheduler(long rob_size, long instr_queue_size, long width, long cache_size, long prefetch, FILE * pFile);
	~InstructionScheduler();

	// Pipeline functions
	void fetch();
	void decode();
	void rename();
	void regRead();
	void dispatch();
	void issue();
	void execute();
	void writeback();
	void retire();

	void print();
	void printResults();

private:
	// helper functions
	int nextIndexInIq();
	int numberOfVacantEntriesInIQ();
	int findOldestInstructionInIQ();
	int nextIndexInEx();
	int nextIndexInWb();
	int nextIndexInRt();

	// Instruction cache helper
	void sendToDe();
};

InstructionScheduler::InstructionScheduler(long rob_size, long instr_queue_size, long width, long cache_size, long prefetch, FILE * pFile)
{
	this->rob_size = rob_size;
	this->instr_queue_size = instr_queue_size;
	this->width = width;
	this->cache_size = cache_size;
	this->prefetch = prefetch;
	this->pFile = pFile;

	rob = new rob_entry[rob_size];
	lRobHead = lRobTail = 0;
	lVacancyInRob = rob_size;

	for (int i = 0; i < 67; i++)
	{
		rmt[i].bValid = false;
	}

	for (int i = 0; i < rob_size; i++)
		rob[i].bReady = false;

	de = new DE[width];
	rn = new RN[width];
	rr = new RR[width];
	di = new DI[width];
	for (int i = 0; i < width; i++)
	{
		de[i].bValid = false;
		rn[i].bValid = false;
		rr[i].bValid = false;
		di[i].bValid = false;
	}
	iq = new IQ[instr_queue_size];
	for (int i = 0; i < instr_queue_size; i++)
	{
		iq[i].bValid = false;
	}
	ex = new EX[width * 5];
	wb = new WB[width * 5];
	for (int i = 0; i < width*5; i++)
	{
		ex[i].bValid = false;
		wb[i].bValid = false;
	}

	simOutput = new SimOutput[MAX_TRACE_FILE_SIZE]; // 10000 - MAX_TRACE_FILE_SIZE - TO_DO

	lSimulatorCycle = 0;
	bTraceDepleted = false;

	// ICache 
	if (cache_size > 0)
	{
		ICache = new Cache(1, 64, cache_size, 4, 0, 0);
		ICache->Init();
	}
	timer = 0;
	bFetchInProgress = false;
	iFetchedInstr = 0;
	bStall = false;
}

InstructionScheduler::~InstructionScheduler()
{
	delete de;
	delete rn;
	delete rr;
	delete di;
	delete iq;
	delete ex;
	delete wb;
	delete rob;
	delete rmt;
	delete simOutput;
}

// Used in dispatch stage
// Next available index is used to place instruction in iq
int InstructionScheduler::nextIndexInIq()
{
	int iVacantEntryInIQ = -1;
	for (int i = 0; i < instr_queue_size; i++)
	{
		if (iq[i].bValid == false)
		{
			iVacantEntryInIQ = i;
			break;
		}
	}
	return iVacantEntryInIQ;
}

int InstructionScheduler::numberOfVacantEntriesInIQ()
{
	int iNumOfVacantEntriesInIq = 0;
	for (int i = 0; i < instr_queue_size; i++)
	{
		if (iq[i].bValid == false)
		{
			iNumOfVacantEntriesInIq++;
		}
	}
	return iNumOfVacantEntriesInIq;
}

// Used in issue stage
// The oldest instruction is removed from iq
int InstructionScheduler::findOldestInstructionInIQ()
{
	int iOldestInstr = -1;
	int iValidInstr = 0;
	for (int i = 0; i < instr_queue_size; i++)
	{
		if (iq[i].bValid && iq[i].bSrc1Rdy && iq[i].bSrc2Rdy)
		{
			iValidInstr++;
			if (iValidInstr == 1)
				iOldestInstr = i;

			if (iq[i].lSeqNum < iq[iOldestInstr].lSeqNum)
				iOldestInstr = i;
		}
	}
	return iOldestInstr;
}

int InstructionScheduler::nextIndexInEx()
{
	int nextIndex = -1;
	for (int i = 0; i < width * 5; i++)
	{
		if (ex[i].bValid == false)
		{
			nextIndex = i;
			break;
		}
	}
	return nextIndex;
}

int InstructionScheduler::nextIndexInWb()
{
	int nextIndex = -1;
	for (int i = 0; i < width * 5; i++)
	{
		if (wb[i].bValid == false)
		{
			nextIndex = i;
			break;
		}
	}
	return nextIndex;
}

bool InstructionScheduler::advance_cycle()
{
	// Advance simulator cycle
	lSimulatorCycle++;
	
	// When pipeline is empty, and trace file is depleted, return false
	bool bPipelineEmpty = true;
	if (bTraceDepleted)
	{
		// check if pipeline is empty
		for (int i = 0; i < width; i++)
		{
			if (de[i].bValid || rn[i].bValid || rr[i].bValid || di[i].bValid)
				bPipelineEmpty = false;
		}
		for (int i = 0; i < instr_queue_size; i++)
		{
			if (iq[i].bValid)
				bPipelineEmpty = false;
		}
		for (int i = 0; i < width * 5; i++)
		{
			if (ex[i].bValid || wb[i].bValid)
				bPipelineEmpty = false;
		}
		if (lRobHead != lRobTail)
		{
			bPipelineEmpty = false;
		}
	}
	/*if (bTraceDepleted)
		cout << lSimulatorCycle << " " << bPipelineEmpty;
*/
	return !(bTraceDepleted && bPipelineEmpty);
}

void InstructionScheduler::sendToDe()
{
	// Set the de fields only after fetched instr count == width
	//if (iFetchedInstr == width)
	//{
		for (int i = 0; i < width; i++)
		{
			de[i].bValid = true;
			de[i].lBeginCycle = lSimulatorCycle;
			de[i].iDuration = 1;
			de[i].lSeqNum = glSeqNumCtr;

			simOutput[glSeqNumCtr].de = de[i];
			simOutput[glSeqNumCtr].lSeqNum = glSeqNumCtr;

			glSeqNumCtr++; // This is the only place this is incremented
		}

		// Reset the fetched isntr counter
		iFetchedInstr = 0;
//	}
}

// Don't fetch if instruction cache is perfect (CACHE_SIZE=0) and either
// (1) there are no more instructions in the trace file or
// (2) DE is not empty (cannot accept a new decode bundle).
// If there are more instructions in the trace file and if DE is empty
// (can accept a new decode bundle), then fetch up to WIDTH
// instructions from the trace file into DE. Fewer than WIDTH
// instructions will be fetched only if the trace file has fewer than
// WIDTH instructions left.
void InstructionScheduler::fetch()
{
	// Check if decode register is empty
	bool bDeEmpty = true;
	for (int i = 0; i < width; i++)
	{
		if (de[i].bValid == true)
		{
			bDeEmpty = false;
		}
	}

	if ((cache_size == 0 || (cache_size != 0 && bFetchInProgress == false)) && (bDeEmpty == false || bTraceDepleted == true))
	{
		// For imperfect cache if we do not have any fetch in progress then fetch should return
		// cout << "Stall in fetch\n";
		return;
	}

	if (cache_size == 0 && prefetch == 0)
	{
		// Insert width number of instr into de
		for (int i = 0; i < width; i++)
		{
			// Just read the string from file
			// One line at a time
			char strBuffer[32];
			if (fgets(strBuffer, 32, pFile) != NULL)
			{
				de[i].bValid = true;
				strcpy(de[i].strInstruction, strBuffer);
				de[i].lBeginCycle = lSimulatorCycle;
				de[i].iDuration = 1;
				de[i].lSeqNum = glSeqNumCtr;

				simOutput[glSeqNumCtr].de = de[i];
				simOutput[glSeqNumCtr].lSeqNum = glSeqNumCtr;

				glSeqNumCtr++; // This is the only place this is incremented
			}
			else
			{
				bTraceDepleted = true;
			}
			//cout << "fetch | " << lSimulatorCycle << " | " << de[i].lSeqNum << " | " << de[i].strInstruction;
		}
	}
	else if ( prefetch == 0 && cache_size != 0)
	{
		// Imperfect cache implementation
		// Non-zero timer means there is an instr being fetched
		// Decrement the counter and return
		if (timer != 0)
		{
			timer--;
			return;
		}

		// Zero timer can mean two things
		// 1. this fetch we have to read file and send instr to de
		// 2. a fetch was in progress and has completed

		// For type 2, we have to reset the flag and increment fetched instruction count
		if (bFetchInProgress)
		{
			bFetchInProgress = false;
			iFetchedInstr++;
			// All instr are fetched, send to de
			if (iFetchedInstr == width)
			{
				sendToDe();
				return;
			}
		}

		// We can have some instructions fetched or it can be a new fetch
		// Fetch instr counter tells us how many instr have been fetched
		// So we fetch only the remaining instrs
		for (int i = iFetchedInstr; i < width; i++)
		{
			char strBuffer[32];
			// Read file
			if (fgets(strBuffer, 32, pFile) != NULL)
			{
				int address, op_type, dest, src1, src2, mask = 0;
				// Find pc 
				strcpy(de[i].strInstruction, strBuffer);
				sscanf(de[i].strInstruction, "%x %d %d %d %d", &address, &op_type, &dest, &src1, &src2);

				// mask for index
				for (int i = 0; i < (int)ICache->m_ullIndexWidth; i++)
				{
					mask <<= 1;
					mask |= 1;
				}

				// Find tag and index
				int index = (address >> ICache->m_ullOffsetWidth) & mask;
				int tag = address >> (ICache->m_ullOffsetWidth + ICache->m_ullIndexWidth); // TO_DO - check this 
				// tag = address >> ICache->m_ullOffsetWidth;

				// Cache Read
				bool bCacheMiss = ICache->Read(index, tag);

				// If Cache hit, do the same thing as we did for perfect cache
				if (!bCacheMiss)
				{
					iFetchedInstr++;
					if (iFetchedInstr == width)
					{
						// All the instructions have been fetched
						// populate De
						sendToDe();
						return;
					}
				}
				else
				{
					// If cache miss, 
					// set a counter 
					timer = 9;

					// Set a flag to indicate there was a miss
					bFetchInProgress = true;

					// Do not read any other instruction
					return;
				}
			}
			else
			{
				bTraceDepleted = true;
			}
		}
	}
	else if (prefetch == 1 && cache_size != 0)
	{
		// Prefetch with instruction cache
		
		if (bFetchInProgress)
		{
			// Decrement the counters
			timer--;
			bFetchInProgress = (timer == 0) ? false : true;
			
			//prefetchTimer--;
			//bPrefetchInProgress = (prefetchTimer == 0) ? false : true;
			// Instruction cannot be ready
			//return;
		}

		// Reduce the prefetch counter independent of anything else
		ICache->reducePrefetchCount();

		for (int i = iFetchedInstr; i < width; i++)
		{
			char strBuffer[32];
			int op_type, dest, src1, src2, mask = 0;
			// Read file
			if (!bStall)
			{
				if (fgets(strBuffer, 32, pFile) != NULL)
				{
					// Find pc 
					strcpy(de[i].strInstruction, strBuffer);
					sscanf(de[i].strInstruction, "%x %d %d %d %d", &address, &op_type, &dest, &src1, &src2);

					//iFetchedInstr++;
				}
				else
				{
					bTraceDepleted = true;
					break;
				}
			}
				// mask for index
			for (int i = 0; i < (int)ICache->m_ullIndexWidth; i++)
			{
				mask <<= 1;
				mask |= 1;
			}

				// Find tag and index
				unsigned long long ullIndex = (address >> ICache->m_ullOffsetWidth) & mask;
				unsigned long long ullTag = address >> (ICache->m_ullOffsetWidth + ICache->m_ullIndexWidth);

				// Cache Read
				bool bCachePlaced = false;
				bool bCacheMiss = false; // When bReturn is true it means read miss.
				// Increment Read Counter
				ICache->m_ullReadCounter++;

				for (int col = 0; col < ICache->m_iAssoc; ++col)
				{
					if (ICache->m_ppbValidityFlag[(int)ullIndex][col] == false && bFetchInProgress == false)
					{
						// We found an invalid line. Use it!
						ICache->m_ppullTagArray[(int)ullIndex][col] = ullTag;

						// Reset its validity flag
						ICache->m_ppbValidityFlag[(int)ullIndex][col] = true;

						// This is a cold miss
						ICache->m_pullReadMissCounter[(int)ullIndex] ++;

						// Update placed flag 
						bCachePlaced = true;

						// We need to send this tag to next level also
						bCacheMiss = true;

						// Update the access counter - lru
						if (ICache->m_iRepPolicy == 0)
						{
							ICache->m_ppullLRUCacheAccessCounter[(int)ullIndex][col] = ICache->FindCountOfMRU(ullIndex) + 1;
						}

						// Prefetch
						int prefetchAdress = (address >> ICache->m_ullOffsetWidth) + 1;
						int prefetchIndex = prefetchAdress & mask;
						int prefetchTag = prefetchAdress >> ICache->m_ullIndexWidth;

						// Call prefetch
						ICache->Prefetch(prefetchIndex, prefetchTag);

						bPrefetchInProgress = true;

						// Counters for fetch
						bFetchInProgress = true;

						timer = 10;

						bStall = true;
						break;
					}
					else
					{
						if (ICache->m_ppullTagArray[(int)ullIndex][col] == ullTag &&
							((/*prefetch*/ICache->m_ppbPrefetched[(int)ullIndex][col] && ICache->m_ppiPrefetchTimer[(int)ullIndex][col] == 0) ||
							(!ICache->m_ppbPrefetched[(int)ullIndex][col] && timer == 0))
							)
						{
							
							// Send to de
							sendToDe();

							// Its a hit
							ICache->m_pullReadHitCounter[(int)ullIndex] ++;

							// We dont need to send this tag to next level also
							bCacheMiss = false;

							// New - If block being read is prefetched then increment count
							if (ICache->m_ppbPrefetched[(int)ullIndex][col] == true)
							{
								if(!bStall)
									ICache->m_iPrefetchedHitCount++;

								ICache->m_ppbPrefetched[(int)ullIndex][col] = false;

								// start prefetching
								int prefetchAdress = (address >> ICache->m_ullOffsetWidth) + 1;
								int prefetchIndex = prefetchAdress & mask;
								int prefetchTag = prefetchAdress >> ICache->m_ullIndexWidth;

								// Call prefetch
								ICache->Prefetch(prefetchIndex, prefetchTag);

								bPrefetchInProgress = true;
							}

							// Update the access counter - lru
							if (ICache->m_iRepPolicy == 0)
							{
								ICache->m_ppullLRUCacheAccessCounter[(int)ullIndex][col] = ICache->FindCountOfMRU(ullIndex) +1;
							}

							// Update placed flag 
							bCachePlaced = true;

							// An hit means fetching needs to restart
							bFetchInProgress = false;
							timer = 0;
							bStall = false;
							break;
						}
					}
				}

				if (bCachePlaced == false && bFetchInProgress == false)
				{
					bStall = true;

					// Prefetch
					int prefetchAdress = (address >> ICache->m_ullOffsetWidth) + 1;
					int prefetchIndex = prefetchAdress & mask;
					int prefetchTag = prefetchAdress >> (ICache->m_ullOffsetWidth + ICache->m_ullIndexWidth);

					// Call prefetch
					ICache->Prefetch(prefetchIndex, prefetchTag);

					bPrefetchInProgress = true;

					// Counters for fetch
					bFetchInProgress = true;

					timer = 10;

					// Its a miss
					ICache->m_pullReadMissCounter[(int)ullIndex] ++;

					// We need to send this tag to next level also
					bCacheMiss = true;

					if (ICache->m_iRepPolicy == 0)
					{
						// LRU implementation
						int col;

						// Find the minimum counter, max count, min counter column number
						unsigned long long ullMin = ICache->m_ppullLRUCacheAccessCounter[(int)ullIndex][0];
						unsigned long long ullMax = ICache->m_ppullLRUCacheAccessCounter[(int)ullIndex][0];

						int iMinCol = 0;
						for (col = 1; col < ICache->m_iAssoc; col++)
						{
							if (ullMin > ICache->m_ppullLRUCacheAccessCounter[(int)ullIndex][col])
							{
								ullMin = ICache->m_ppullLRUCacheAccessCounter[(int)ullIndex][col];
								iMinCol = col;
							}
							if (ullMax < ICache->m_ppullLRUCacheAccessCounter[(int)ullIndex][col])
							{
								ullMax = ICache->m_ppullLRUCacheAccessCounter[(int)ullIndex][col];
							}
						}

						ICache->m_ppullTagArray[(int)ullIndex][iMinCol] = ullTag;

						ICache->m_ppbPrefetched[(int)ullIndex][iMinCol] = false;

						// Update Minimum Column's tag
						ICache->m_ppullLRUCacheAccessCounter[(int)ullIndex][iMinCol] = ullMax + 1;
					}
				}
		}

		//if (bPrefetchInProgress)
		//{
		//	prefetchTimer--;
		//	bPrefetchInProgress = (prefetchTimer == 0) ? false : true;
		//}
	}
}

// If DE contains a decode bundle:
// If RN is not empty (cannot accept a new rename bundle), then do
// nothing. If RN is empty (can accept a new rename bundle), then
// advance the decode bundle from DE to RN.
void InstructionScheduler::decode()
{
	// Check if DE contains instruction(s)
	bool bIsDeValid = false;
	for (int i = 0; i < width; i++)
	{
		if (de[i].bValid == true)
		{
			bIsDeValid = true; // Even if we have one valid instruction in de we need to decode it
			break;
		}
	}

	if (bIsDeValid)
	{
		// Check if RN is empty
		bool bRnEmpty = true;
		for (int i = 0; i < width; i++)
		{
			if (rn[i].bValid)
			{
				bRnEmpty = false;
			}
		}

		// Store valid DE into RN
		if (bRnEmpty)
		{
			for (int i = 0; i < width; i++)
			{
				if (de[i].bValid == false)
				{
					break;
				}

				rn[i].bValid = de[i].bValid;
				rn[i].lSeqNum = de[i].lSeqNum;

				// Parse string like a decode instr would
				sscanf(de[i].strInstruction, "%x %d %d %d %d", &rn[i].program_counter, &rn[i].operation_type, &rn[i].dest_reg, &rn[i].src1_reg, &rn[i].src2_reg);
			
				// current stage begin cycle = prev stage begin cycle + prev stage duration
				// current stage duration = current simulator cycle - current begin cycle + 1
				rn[i].lBeginCycle = de[i].lBeginCycle + de[i].iDuration;
				rn[i].iDuration = lSimulatorCycle - rn[i].lBeginCycle + 1;

				simOutput[de[i].lSeqNum].rn = rn[i]; // Always store the stage information in a structure which will be used for tracing info

				// Make DE entry invalid
				de[i].bValid = false;

				//cout << "decode | " << lSimulatorCycle << " | " << rn[i].lSeqNum << " | D = " << rn[i].dest_reg
					//<< " | S1 = " << rn[i].src1_reg << " | S2 = " << rn[i].src2_reg << endl;
			}
		}
	}
}

// If RN contains a rename bundle:
// If either RR is not empty (cannot accept a new register-read bundle)
// or the ROB does not have enough free entries to accept the entire
// rename bundle, then do nothing.
// If RR is empty (can accept a new register-read bundle) and the ROB
// has enough free entries to accept the entire rename bundle, then
// process (see below) the rename bundle and advance it from RN to RR.
//
// How to process the rename bundle:
// Apply your learning from the class lectures/notes on the steps for
// renaming:
// (1) Allocate an entry in the ROB for the instruction,
// (2) Rename its source registers, and
// (3) Rename its destination register (if it has one).
// Note that the rename bundle must be renamed in program order.
// Fortunately, the instructions in the rename bundle are in program
// order).
void InstructionScheduler::rename()
{
	// Check if RN contains instruction(s)
	bool bIsRnValid = false;
	for (int i = 0; i < width; i++)
	{
		if (rn[i].bValid == true)
		{
			bIsRnValid = true; // Even if we have one valid instruction in rn we need to rename it
			break;
		}
	}

	if (bIsRnValid)
	{
		// Check if RR is empty
		bool bRrEmpty = true;
		for (int i = 0; i < width; i++)
		{
			if (rr[i].bValid)
			{
				bRrEmpty = false;
			}
		}

		// Check if Rob has space
		bool bRobHasSpace = false;
		if (lVacancyInRob >= width)
		{
			bRobHasSpace = true;
		}
		else
		{
			bRobHasSpace = false;
		}
		
		// There is space in RR and ROB
		if (bRrEmpty && bRobHasSpace)
		{
			for (int i = 0; i < width; i++)
			{
				if (rn[i].bValid == false)
				{
					break;
				}

				// Fill RR
				rr[i].bValid = rn[i].bValid;
				rr[i].lBeginCycle = rn[i].lBeginCycle + rn[i].iDuration;
				rr[i].lSeqNum = rn[i].lSeqNum;
				rr[i].iDuration = lSimulatorCycle - rr[i].lBeginCycle + 1;
				rr[i].operation_type = rn[i].operation_type;
				rr[i].program_counter = rn[i].program_counter;
				rr[i].rn_dest_reg = lRobTail; // Rename dest register

				// Fill values in rob entry
				rob[lRobTail].ulProgramCtr = rn[i].program_counter;
				rob[lRobTail].bReady = false;
				rob[lRobTail].iDestRegister = rn[i].dest_reg;
				rob[lRobTail].lSeqNum = rn[i].lSeqNum;

				// Search src reg is RMT
				if (rn[i].src1_reg != -1 && rmt[rn[i].src1_reg].bValid)
				{
					// rename src1
					rr[i].bSrc1Rdy = false; // Src found in RMT
					rr[i].rn_src1_reg = rmt[rn[i].src1_reg].lRobTag;
				}
				else
				{
					rr[i].bSrc1Rdy = true;
					rr[i].rn_src1_reg = rn[i].src1_reg;
				}

				if (rn[i].src2_reg != -1 && rmt[rn[i].src2_reg].bValid)
				{
					// rename src2
					rr[i].bSrc2Rdy = false;
					rr[i].rn_src2_reg = rmt[rn[i].src2_reg].lRobTag;
				}
				else
				{
					rr[i].bSrc2Rdy = true;
					rr[i].rn_src2_reg = rn[i].src2_reg;
				}

				// Update RMT
				if (rob[lRobTail].iDestRegister >= 0)
				{
					rmt[rob[lRobTail].iDestRegister].bValid = true; // Destination register is index of rmt
					rmt[rob[lRobTail].iDestRegister].lRobTag = lRobTail;
				}

				lRobTail++;
				if (lRobTail == rob_size)
				{
					lRobTail = 0;
				}
				lVacancyInRob--;

				simOutput[rn[i].lSeqNum].rr = rr[i];

				rn[i].bValid = false; // Reset previous stage
				/*cout << "rename | " << lSimulatorCycle << " | " << rr[i].lSeqNum << " | D = " << rr[i].rn_dest_reg 
					<< " | S1 = " << rr[i].rn_src1_reg << " | S2 = " << rr[i].rn_src2_reg
					<< " | S1 rdy = " << rr[i].bSrc1FoundInARF << " | S2 rdy = " << rr[i].bSrc2FoundInARF << endl;*/
			}
		}
	}
}

// If RR contains a register-read bundle:
// If DI is not empty (cannot accept a new dispatch bundle), then do
// nothing. If DI is empty (can accept a new dispatch bundle), then
// process (see below) the register-read bundle and advance it from RR
// to DI.
//
// How to process the register-read bundle:
// Since values are not explicitly modeled, the sole purpose of the
// Register Read stage is to ascertain the readiness of the renamed
// source operands. Apply your learning from the class lectures/notes
// on this topic.
//
// Also take care that producers in their last cycle of execution
// wakeup dependent operands not just in the IQ, but also in two other
// stages including RegRead()(this is required to avoid deadlock). See
// Execute() description above.
void InstructionScheduler::regRead()
{
	// Check if RR contains instruction(s)
	bool bIsRrValid = false;
	for (int i = 0; i < width; i++)
	{
		if (rr[i].bValid == true)
		{
			bIsRrValid = true; // Even if we have one valid instruction in rr we need to read it
			break;
		}
	}
	if (bIsRrValid)
	{
		// Check if RN is empty
		bool bDiEmpty = true;
		for (int i = 0; i < width; i++)
		{
			if (di[i].bValid)
			{
				bDiEmpty = false;
			}
		}

		// Store valid RR into DI
		if (bDiEmpty)
		{
			for (int i = 0; i < width; i++)
			{
				if (rr[i].bValid == false)
				{
					break;
				}
				di[i].bValid = rr[i].bValid;
				di[i].lBeginCycle = rr[i].lBeginCycle + rr[i].iDuration;
				di[i].lSeqNum = rr[i].lSeqNum;
				di[i].operation_type = rr[i].operation_type;
				di[i].program_counter = rr[i].program_counter;
				di[i].rn_dest_reg = rr[i].rn_dest_reg;
				di[i].rn_src1_reg = rr[i].rn_src1_reg;
				di[i].rn_src2_reg = rr[i].rn_src2_reg;
				di[i].iDuration = lSimulatorCycle - di[i].lBeginCycle + 1;

				di[i].bSrc1Rdy = di[i].bSrc2Rdy = false;
				if (rr[i].bSrc1Rdy)
				{
					di[i].bSrc1Rdy = true;
				}
				else
				{
					// If source is not found in ARF, it means that source is present in ROB
					// Check if it is ready in Rob
					// Assign the status of Rob to di's status
					di[i].bSrc1Rdy = rob[rr[i].rn_src1_reg].bReady;
				}
				
				if (rr[i].bSrc2Rdy)
				{
					di[i].bSrc2Rdy = true;
				}
				else
				{
					di[i].bSrc2Rdy = rob[rr[i].rn_src2_reg].bReady;
				}
				simOutput[rr[i].lSeqNum].di = di[i];
				rr[i].bValid = false; 
				/*cout << "reg read | " << lSimulatorCycle << " | " << di[i].lSeqNum << " | D = " << di[i].rn_dest_reg
					<< " | S1 = " << di[i].rn_src1_reg << " | S2 = " << di[i].rn_src2_reg
					<< " | S1 rdy = " << di[i].bSrc1Rdy << " | S2 rdy = " << di[i].bSrc2Rdy << endl;
				*/
			}
		}
	}
}

// If DI contains a dispatch bundle:
// If the number of free IQ entries is less than the size of the
// dispatch bundle in DI, then do nothing. If the number of free IQ
// entries is greater than or equal to the size of the dispatch bundle
// in DI, then dispatch all instructions from DI to the IQ.
void InstructionScheduler::dispatch()
{
	// Check if DI contains instruction(s)
	int iSizeOfDI = 0;
	for (int i = 0; i < width; i++)
	{
		if (di[i].bValid == true)
		{
			iSizeOfDI++;
		}
	}

	if (iSizeOfDI > 0)
	{
		// Check if IQ has space
		int iVacancyInIQ = numberOfVacantEntriesInIQ();
		if (iVacancyInIQ < iSizeOfDI)
		{
			// Do nothing
			// cout << "No vacancy in IQ\n";
			return;
		}
		
		for (int i = 0; i < iSizeOfDI; i++)
		{
			int iVacantEntry = nextIndexInIq();
			if (iVacantEntry >= 0)
			{
				iq[iVacantEntry].bValid = di[i].bValid;
				iq[iVacantEntry].bSrc1Rdy = di[i].bSrc1Rdy;
				iq[iVacantEntry].bSrc2Rdy = di[i].bSrc2Rdy;
				iq[iVacantEntry].iDest = di[i].rn_dest_reg;
				iq[iVacantEntry].iSrc1 = di[i].rn_src1_reg;
				iq[iVacantEntry].iSrc2 = di[i].rn_src2_reg;
				iq[iVacantEntry].lSeqNum = di[i].lSeqNum;
				iq[iVacantEntry].operation_type = di[i].operation_type;
				iq[iVacantEntry].lBeginCycle = di[i].lBeginCycle + di[i].iDuration;
				iq[iVacantEntry].program_counter = di[i].program_counter;
				iq[iVacantEntry].iDuration = lSimulatorCycle - iq[iVacantEntry].lBeginCycle + 1;

				
				simOutput[di[i].lSeqNum].iq = iq[iVacantEntry];
				di[i].bValid = false;
			}
			else
			{
				// TO_DO: comment this
				cout << "Something is wrong.. No vacancy in IQ\n" << endl;
			}
		}
	}
}

// Issue up to WIDTH oldest instructions from the IQ. (One approach to
// implement oldest-first issuing is to make multiple passes through
// the IQ, each time finding the next oldest ready instruction and then
// issuing it. One way to annotate the age of an instruction is to
// assign an incrementing sequence number to each instruction as it is
// fetched from the trace file.)
// To issue an instruction:
// 1) Remove the instruction from the IQ.
// 2) Add the instruction to the execute_list. Set a timer for
// the instruction in the execute_list that will allow you to
// model its execution latency.
void InstructionScheduler::issue()
{
	// Check if IQ contains instruction(s)
	int iSizeOfIq = 0;
	for (int i = 0; i < instr_queue_size; i++)
	{
		if (iq[i].bValid == true)
		{
			iSizeOfIq++;
			//cout << "IQ------\n";
			//cout << i << ". Src1Rdy = " << iq[i].bSrc1Rdy << " Src2Rdy = " << iq[i].bSrc2Rdy << " Src1 = " << iq[i].iSrc1 << " Src2 = " << iq[i].iSrc2 << endl;
		}
	}

	if (iSizeOfIq > 0)
	{
		// Ideally we should not check in EX for empty space
		// EX must accept width instruction every cycle
		// Doing this just in case it is needed later..
		// Check if EX is empty
		int iVacancyInEx = 0;
		for (int i = 0; i < width*5; i++)
		{
			if (ex[i].bValid == false)
			{
				iVacancyInEx++;
			}
		}

		if (iVacancyInEx < width)
		{
			// cout << "Something went wrong.. We do not have vacancy in EX\n";
		}

		// Issue upto width instr
		for (int i = 0; i < width; i++)
		{
			int iOldestInstrIdx = findOldestInstructionInIQ();
			int iNextAvailableIndexInEx = nextIndexInEx();
			
			if (iOldestInstrIdx != -1)
			{
				// Add to EX list
				if (iNextAvailableIndexInEx == -1)
				{
					// cout << "Something went wrong.. isn't it obvious why\n";
					return;
				}
				
				ex[iNextAvailableIndexInEx].bValid = iq[iOldestInstrIdx].bValid;
				ex[iNextAvailableIndexInEx].iDest = iq[iOldestInstrIdx].iDest;
				ex[iNextAvailableIndexInEx].iSrc1 = iq[iOldestInstrIdx].iSrc1;
				ex[iNextAvailableIndexInEx].iSrc2 = iq[iOldestInstrIdx].iSrc2;
				ex[iNextAvailableIndexInEx].program_counter = iq[iOldestInstrIdx].program_counter;
				ex[iNextAvailableIndexInEx].lBeginCycle = iq[iOldestInstrIdx].lBeginCycle + iq[iOldestInstrIdx].iDuration;
				ex[iNextAvailableIndexInEx].iDuration = lSimulatorCycle - ex[iNextAvailableIndexInEx].lBeginCycle + 1;
				ex[iNextAvailableIndexInEx].lSeqNum = iq[iOldestInstrIdx].lSeqNum;
				ex[iNextAvailableIndexInEx].operation_type = iq[iOldestInstrIdx].operation_type;
					
				switch (iq[iOldestInstrIdx].operation_type)
				{
				case 0:
					ex[iNextAvailableIndexInEx].iExecuteLatency = 1;
					break;
				case 1:
					ex[iNextAvailableIndexInEx].iExecuteLatency = 2;
					break;
				case 2:
					ex[iNextAvailableIndexInEx].iExecuteLatency = 5;
					break;
				default:
					// cout << "Incorrect operation type";
					break;
				}
				
				simOutput[iq[iOldestInstrIdx].lSeqNum].ex = ex[iNextAvailableIndexInEx];

				iq[iOldestInstrIdx].bValid = false; // Removing instr from IQ
			}
			else
			{
				// Ideally, we should not be here because size of iq will ensure 
				// that we only run the loop for valid instructions
				// cout << "No valid instruction in IQ\n";
				break;
			}
		}
	}
}

// From the execute_list, check for instructions that are finishing
// execution this cycle, and:
// 1) Remove the instruction from the execute_list.
// 2) Add the instruction to WB.
// 3) Wakeup dependent instructions (set their source operand ready
// flags) in the IQ, DI (dispatch bundle), and RR (the register-read
// bundle).
void InstructionScheduler::execute()
{
	// There will not be a stall check in ex and wb stage. 
	// Each instr can be processed by these stages
	for (int i = 0; i < width * 5; i++)
	{
		if (ex[i].bValid)
		{
			ex[i].iExecuteLatency--;
			if (ex[i].iExecuteLatency == 0)
			{
				// Add to WB
				int iNextAvailableIndexInWb = nextIndexInWb();
				if (iNextAvailableIndexInWb == -1)
				{
					// cout << "Something went wrong during EX..\n";
					return;
				}
				
				// Insert into Wb stage
				wb[iNextAvailableIndexInWb].bValid = ex[i].bValid;
				wb[iNextAvailableIndexInWb].iDest = ex[i].iDest;
				wb[iNextAvailableIndexInWb].iSrc1 = ex[i].iSrc1;
				wb[iNextAvailableIndexInWb].iSrc2 = ex[i].iSrc2;
				wb[iNextAvailableIndexInWb].program_counter = ex[i].program_counter;
				wb[iNextAvailableIndexInWb].lBeginCycle = ex[i].lBeginCycle + ex[i].iDuration;
				wb[iNextAvailableIndexInWb].iDuration = lSimulatorCycle - wb[iNextAvailableIndexInWb].lBeginCycle + 1;
				wb[iNextAvailableIndexInWb].operation_type = ex[i].operation_type;
				wb[iNextAvailableIndexInWb].lSeqNum = ex[i].lSeqNum;

				simOutput[ex[i].lSeqNum].wb = wb[iNextAvailableIndexInWb];
				
				// Imp - send wakeup signal
				// int iSeqNum = ex[i].lSeqNum;
				int iReg = ex[i].iDest;
				// cout << "Waking up.. Seq Num = " << iSeqNum << " Reg = " << iReg << endl;
				for (int j = 0; j < instr_queue_size; j++)
				{
					if (iq[j].bValid)
					{
						if (iReg == iq[j].iSrc1)
						{
							// cout << "Wake up iq.. j = " << j << " Src1 reg = " << iq[j].iSrc1 << endl;
							iq[j].bSrc1Rdy = true;
						}
						if (iReg == iq[j].iSrc2)
						{
							// cout << "Wake up iq.. j = " << j << " Src2 = " << iq[j].iSrc2 << endl;
							iq[j].bSrc2Rdy = true;
						}
					}
				}

				for (int j = 0; j < width; j++)
				{
					if (di[j].bValid)
					{
						if (iReg == di[j].rn_src1_reg)
						{
							// cout << "Wake up Di.. j = " << j << " Src1 reg = " << di[j].rn_src1_reg << " Src2 = " << di[j].rn_src2_reg << endl;
							di[j].bSrc1Rdy = true;
						}
						if (iReg == di[j].rn_src2_reg)
						{
							// cout << "Wake up Di.. j = " << j << " Src1 reg = " << di[j].rn_src1_reg << " Src2 = " << di[j].rn_src2_reg << endl;
							di[j].bSrc2Rdy = true;
						}
					}
					
					if (rr[j].bValid)
					{
						if (iReg == rr[j].rn_src1_reg)
						{
							// cout << "Wake up rr.. j = " << j << " Src1 reg = " << rr[j].rn_src1_reg << " Src2 = " << rr[j].rn_src2_reg << endl;
							rr[j].bSrc1Rdy = true;
						}
						if (iReg == rr[j].rn_src2_reg)
						{
							// cout << "Wake up rr.. j = " << j << " Src1 reg = " << rr[j].rn_src1_reg << " Src2 = " << rr[j].rn_src2_reg << endl;
							rr[j].bSrc2Rdy = true;
						}
					}
				}

				// Remove from execute list
				ex[i].bValid = false;
			}
		}
	}
}

// Process the writeback bundle in WB: For each instruction in WB, mark
// the instruction as “ready” in its entry in the ROB.
void InstructionScheduler::writeback()
{
	for (int i = 0; i < width * 5; i++)
	{
		if (wb[i].bValid)
		{
			rob[wb[i].iDest].bReady = true;

			//rt[iRtIdx].bValid = wb[i].bValid;
			//rt[iRtIdx].program_counter = wb[i].program_counter;
			//rt[iRtIdx].lSeqNum = wb[i].lSeqNum;
			// Not sure if all the fields are needed

			//cout << "i= " << iRtIdx << "seq= " << rt[iRtIdx].lSeqNum << " | ";

			simOutput[wb[i].lSeqNum].lWritebackBeginCycle = wb[i].lBeginCycle + wb[i].iDuration;
			simOutput[wb[i].lSeqNum].iWriteBackDuration = lSimulatorCycle - simOutput[wb[i].lSeqNum].lWritebackBeginCycle + 1;
			wb[i].bValid = false;
			
			if (wb[i].lSeqNum == 9997)
			{
				//cout << "execute" << endl;
			}
			if (wb[i].iDest == 40 && wb[i].lSeqNum > 9930 && wb[i].lSeqNum < 10000)
			{
				//cout << wb[i].lSeqNum << " | ";
			}
		}
	}
	//cout << endl;
}

// Retire up to WIDTH consecutive “ready” instructions from the head of
// the ROB.
void InstructionScheduler::retire()
{
	for (int i = 0; i < width; i++)
	{
		if (rob[lRobHead].bReady)
		{
			// Move the instruction out of the pipeline
			if (rob[lRobHead].lSeqNum == glRetireCtr)
			{
				simOutput[glRetireCtr].lRetireBeginCycle = simOutput[glRetireCtr].lWritebackBeginCycle + simOutput[glRetireCtr].iWriteBackDuration;
				simOutput[glRetireCtr].iRetireDuration = lSimulatorCycle - simOutput[glRetireCtr].lRetireBeginCycle + 1;
				glRetireCtr++;
				//cout << "i= " << i << " seq = " << seq << " retired | ";

				// Update RMT
				int iRmtIndex = rob[lRobHead].iDestRegister;
				if (iRmtIndex != -1 && rmt[iRmtIndex].lRobTag == lRobHead)
				{
					rmt[iRmtIndex].bValid = false;
				}

				// Update rob head
				lRobHead++;
				lVacancyInRob++;
				if (lRobHead == rob_size)
				{
					lRobHead = 0;
					// cout << "Rob head = " << lRobHead << " Tail = " << lRobTail << endl;
				}
/*
				int seq = glRetireCtr - 1;
				printf("%d fu{%d} src{%d,%d} dst{%d} FE{%d,%d} DE{%d,%d} RN{%d,%d} RR{%d,%d} DI{%d,%d} IS{%d,%d} EX{%d,%d} WB{%d,%d} RT{%d,%d}\n",
					simOutput[seq].lSeqNum, simOutput[seq].rn.operation_type, simOutput[seq].rn.src1_reg, simOutput[seq].rn.src2_reg, simOutput[seq].rn.dest_reg,
					simOutput[seq].de.lBeginCycle, simOutput[seq].de.iDuration,
					simOutput[seq].rn.lBeginCycle, simOutput[seq].rn.iDuration,
					simOutput[seq].rr.lBeginCycle, simOutput[seq].rr.iDuration,
					simOutput[seq].di.lBeginCycle, simOutput[seq].di.iDuration,
					simOutput[seq].iq.lBeginCycle, simOutput[seq].iq.iDuration,
					simOutput[seq].ex.lBeginCycle, simOutput[seq].ex.iDuration,
					simOutput[seq].wb.lBeginCycle, simOutput[seq].wb.iDuration,
					simOutput[seq].lWritebackBeginCycle, simOutput[seq].iWriteBackDuration,
					simOutput[seq].lRetireBeginCycle, simOutput[seq].iRetireDuration
					);*/
			}
		}
		else
		{
			// cout << "No more instructions to be retired.\n";
			break;
		}
	}
}

void InstructionScheduler::print()
{
	for (int i = 0; i < glSeqNumCtr; i++)
	{
		printf("%li fu{%d} src{%d,%d} dst{%d} FE{%li,%d} DE{%li,%d} RN{%li,%d} RR{%li,%d} DI{%li,%d} IS{%li,%d} EX{%li,%d} WB{%li,%d} RT{%li,%d}\n",
			simOutput[i].lSeqNum, simOutput[i].rn.operation_type, simOutput[i].rn.src1_reg, simOutput[i].rn.src2_reg, simOutput[i].rn.dest_reg, 
			simOutput[i].de.lBeginCycle, simOutput[i].de.iDuration,
			simOutput[i].rn.lBeginCycle, simOutput[i].rn.iDuration,
			simOutput[i].rr.lBeginCycle, simOutput[i].rr.iDuration,
			simOutput[i].di.lBeginCycle, simOutput[i].di.iDuration,
			simOutput[i].iq.lBeginCycle, simOutput[i].iq.iDuration,
			simOutput[i].ex.lBeginCycle, simOutput[i].ex.iDuration, 
			simOutput[i].wb.lBeginCycle, simOutput[i].wb.iDuration, 
			simOutput[i].lWritebackBeginCycle, simOutput[i].iWriteBackDuration, 
			simOutput[i].lRetireBeginCycle, simOutput[i].iRetireDuration
			);
	}
}

void InstructionScheduler::printResults()
{
	printf("# === Simulation Results ========\n");
	printf("# Dynamic Instruction Count      = %li\n", glSeqNumCtr);
	printf("# Cycles                         = %li\n", lSimulatorCycle);
	printf("# Instructions Per Cycle (IPC)   = %.2f\n", (float)glSeqNumCtr / lSimulatorCycle);
	if (cache_size > 0 && prefetch == 1)
	{
		printf("# Instruction Cache Hits:      = %llu\n", ICache->calculateReadHits() - ICache->calculateReadMisses());
		printf("# Prefetch Hits:               =  %d\n", ICache->m_iPrefetchedHitCount);
	}
	if (cache_size > 0 && prefetch == 0)
	{
		printf("# Instruction Cache Hits:      = %llu\n", ICache->calculateReadHits());
	}
}

int main(int argc, char* argv[])
{
	long rob_size, instr_queue_size, width, cache_size, prefetch;
	char* tracefile;
	FILE * pFile;

	if (argc != 7)
	{
		return 1;
	}

	rob_size = atoi(argv[1]);
	instr_queue_size = atoi(argv[2]);
	width = atoi(argv[3]);
	cache_size = atoi(argv[4]);
	prefetch = atoi(argv[5]);
	tracefile = argv[6];

	pFile = fopen(tracefile, "r");
	
	if (pFile == NULL)
	{
		cout << "Incorrect tracefile";
		return 1;
	}

	InstructionScheduler* pobjInstrScheduler = new InstructionScheduler(rob_size, instr_queue_size, width, cache_size, prefetch, pFile);
	
	//ifstream myfile(tracefile, ifstream::in);
	
	//for (int i = 0; !pobjInstrScheduler->bTraceDepleted; i++)
	//{
	do
	{
		pobjInstrScheduler->retire();
		pobjInstrScheduler->writeback();
		pobjInstrScheduler->execute();
		pobjInstrScheduler->issue();
		pobjInstrScheduler->dispatch();
		pobjInstrScheduler->regRead();
		pobjInstrScheduler->rename();
		pobjInstrScheduler->decode();
		pobjInstrScheduler->fetch();

		//cout << "i= " << i << "Rob head = " << pobjInstrScheduler->lRobHead << "Rob tail = " << pobjInstrScheduler->lRobTail << endl;
	} while (pobjInstrScheduler->advance_cycle());
	
	pobjInstrScheduler->print();
	printCommandLine(argc, argv);
	pobjInstrScheduler->printResults();
	fclose(pFile);
	return 0;
}