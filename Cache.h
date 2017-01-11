#include <iostream>
using namespace std;

#include <string>
#include <fstream>
#include <math.h> 
#include <sstream>
#include <bitset>

#include <stdlib.h>
#include <stdio.h>

class Cache
{
	// Variables
	// From Command line
public:
	int m_iBlockSize;
	int m_iSize;
	int m_iAssoc;
	int m_iRepPolicy;
	int m_iInclusion;

	// 
	int m_iLevel;
	int m_iSets;
	unsigned long long m_ullIndexWidth;
	unsigned long long m_ullOffsetWidth;
	unsigned long long m_ullTagWidth;

	// New - prefetched flag
	bool ** m_ppbPrefetched;
	int ** m_ppiPrefetchTimer;
	int ** m_ppiFetchTimer;

	int m_iPrefetchedHitCount;

	// Tag matrix
	unsigned long long ** m_ppullTagArray;

	// How many times a cache element waas accessed
	unsigned long long ** m_ppullLRUCacheAccessCounter;

	// Number of write misses
	unsigned long long * m_pullWriteMissCounter;

	// Number of write hits
	unsigned long long * m_pullWriteHitCounter;

	// Number of write misses
	unsigned long long * m_pullReadMissCounter;

	// Number of write hits
	unsigned long long * m_pullReadHitCounter;

	// Validity flag
	bool ** m_ppbValidityFlag;

	// Dirty bit
	bool ** m_ppbDirtyBit;

	// Write Counter
	unsigned long long m_ullWriteCounter;

	// Read Counter
	unsigned long long m_ullReadCounter;

	// FIFO counter
	unsigned long long ** m_ppullFIFOCounter;

	// Psuedo LRU
	bool ** m_ppbPseudoLRU;

	// Writeback counter - incremented whenever a block with dirty flag set is evicted.
	unsigned long long m_ullWriteBackCounter;

	// Clean L1 evictions
	unsigned long long m_ullCleanEvictions;

	// Functions
public:
	bool Read(unsigned long long ullIndex, unsigned long long ullTag);
	bool Write(unsigned long long ullIndex, unsigned long long ullTag);
	bool Prefetch(unsigned long long ullIndex, unsigned long long ullTag);
	void reducePrefetchCount();
	int m_iPrefetchCounter;

	void Init(); // To set all the important internal variables correctly

	void setNumberOfSets();
	void setIndexWidth();
	void setOffestWidth();
	void setTagWidth();

	unsigned long long FindCountOfMRU(unsigned long long ullIndex);
	unsigned long long FindMaxCountOfFIFO(unsigned long long ullIndex);

	unsigned long long calculateReadMisses();
	unsigned long long calculateReadHits();
	unsigned long long calculateWriteMisses();
	unsigned long long calculateWriteHits();
	float calculateMissRate();

public:
	// Constructor
	Cache();
	Cache(int iLevel, int iBlockSize, int iSize, int iAssoc, int iRepPolicy, int iIncPolicy);
};
