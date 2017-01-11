#include "Cache.h"

Cache::Cache(int iLevel, int iBlockSize, int iSize, int iAssoc, int iRepPolicy, int iIncPolicy)
{
	m_iLevel = iLevel;
	m_iBlockSize = iBlockSize;
	m_iSize = iSize;
	m_iAssoc = iAssoc;
	m_iRepPolicy = iRepPolicy;
	m_iInclusion = iIncPolicy;
}

void Cache::Init()
{
	if (m_iSets != 0 || m_iAssoc != 0)
	{
		setNumberOfSets();
		setIndexWidth();
		setOffestWidth();
		setTagWidth();

		m_ullWriteCounter = 0;
		m_ullReadCounter = 0;

		m_ullWriteBackCounter = 0;
		m_ullCleanEvictions = 0;

		m_ppullTagArray = new unsigned long long*[m_iSets];
		for (int i = 0; i < m_iSets; ++i)
			m_ppullTagArray[i] = new unsigned long long[m_iAssoc];

		m_ppullLRUCacheAccessCounter = new unsigned long long *[m_iSets];
		for (int i = 0; i < m_iSets; ++i)
			m_ppullLRUCacheAccessCounter[i] = new unsigned long long[m_iAssoc];

		m_ppbValidityFlag = new bool*[m_iSets];
		for (int i = 0; i < m_iSets; ++i)
			m_ppbValidityFlag[i] = new bool[m_iAssoc];

		m_ppbDirtyBit = new bool*[m_iSets];
		for (int i = 0; i < m_iSets; ++i)
			m_ppbDirtyBit[i] = new bool[m_iAssoc];

		// New
		m_iPrefetchedHitCount = m_iPrefetchCounter = 0;
		m_ppbPrefetched = new bool*[m_iSets];
		for (int i = 0; i < m_iSets; ++i)
		{
			m_ppbPrefetched[i] = new bool[m_iAssoc];
			for (int j = 0; j < m_iAssoc; j++)
			{
				m_ppbPrefetched[i][j] = false;
			}
		}

		m_ppiPrefetchTimer = new int*[m_iSets];
		m_ppiFetchTimer = new int*[m_iSets];
		for (int i = 0; i < m_iSets; ++i)
		{
			m_ppiPrefetchTimer[i] = new int[m_iAssoc];
			m_ppiFetchTimer[i] = new int[m_iAssoc];
			for (int j = 0; j < m_iAssoc; j++)
			{
				m_ppiPrefetchTimer[i][j] = 0;
				m_ppiFetchTimer[i][j] = 0;
			}
		}

		m_ppullFIFOCounter = new unsigned long long *[m_iSets];
		for (int i = 0; i < m_iSets; ++i)
			m_ppullFIFOCounter[i] = new unsigned long long[m_iAssoc];

		m_pullWriteMissCounter = new unsigned long long[m_iSets];

		m_pullWriteHitCounter = new unsigned long long[m_iSets];

		m_pullReadMissCounter = new unsigned long long[m_iSets];

		m_pullReadHitCounter = new unsigned long long[m_iSets];

		for (int i = 0; i < m_iSets; ++i)
		{
			for (int j = 0; j < m_iAssoc; ++j)
			{
				m_ppullTagArray[i][j] = 0;
				m_ppullLRUCacheAccessCounter[i][j] = 0;
				m_ppbValidityFlag[i][j] = false;
				m_ppullFIFOCounter[i][j] = 0;
				m_ppbDirtyBit[i][j] = false;
			}

			m_pullWriteMissCounter[i] = 0;
			m_pullWriteHitCounter[i] = 0;
			m_pullReadMissCounter[i] = 0;
			m_pullReadHitCounter[i] = 0;
		}

		m_ppbPseudoLRU = new bool*[m_iSize];
		for (int i = 0; i < m_iSize; ++i)
		{
			m_ppbPseudoLRU[i] = new bool[m_iAssoc - 1];
			for (int j = 0; j < m_iAssoc - 1; ++j)
			{
				m_ppbPseudoLRU[i][j] = true;
			}
		}
	}
}

void Cache::setNumberOfSets()
{
	m_iSets = m_iSize / (m_iBlockSize*m_iAssoc);
}

void Cache::setIndexWidth()
{
	m_ullIndexWidth = log2(m_iSets);
}

void Cache::setOffestWidth()
{
	m_ullOffsetWidth = log2(m_iBlockSize);
}

void Cache::setTagWidth()
{
	m_ullTagWidth = 64 - m_ullIndexWidth - m_ullOffsetWidth; // TO_DO: Check if this is correct: 64 - We will always pad the address to 64-bits
}

unsigned long long Cache::calculateReadHits()
{
	// Sum of read hits
	unsigned long long ullTotalHits = 0;
	for (int i = 0; i < m_iSets; ++i)
	{
		ullTotalHits += m_pullReadHitCounter[i];
	}
	return ullTotalHits;
}

unsigned long long Cache::calculateReadMisses()
{
	// Sum of read misses
	unsigned long long ullTotalMissess = 0;
	for (int i = 0; i < m_iSets; ++i)
	{
		ullTotalMissess += m_pullReadMissCounter[i];
	}
	return ullTotalMissess;
}

unsigned long long Cache::calculateWriteHits()
{
	// Sum of write hits
	unsigned long long ullTotalHits = 0;
	for (int i = 0; i < m_iSets; ++i)
	{
		ullTotalHits += m_pullWriteHitCounter[i];
	}
	return ullTotalHits;
}

unsigned long long Cache::calculateWriteMisses()
{
	// Sum of write misses
	unsigned long long ullTotalMissess = 0;
	for (int i = 0; i < m_iSets; ++i)
	{
		ullTotalMissess += m_pullWriteMissCounter[i];
	}
	return ullTotalMissess;
}

float Cache::calculateMissRate()
{
	unsigned long long ullTotalReadMissess = calculateReadMisses();
	unsigned long long ullTotalWriteMisses = calculateWriteMisses();
	float fMissRate = (float)(ullTotalReadMissess + ullTotalWriteMisses) / (m_ullReadCounter + m_ullWriteCounter);

	return fMissRate;
}

unsigned long long Cache::FindCountOfMRU(unsigned long long ullIndex)
{
	unsigned long long ullMax = m_ppullLRUCacheAccessCounter[(int)ullIndex][0];

	// Find mru number
	for (int i = 1; i < m_iAssoc; i++)
	{

		if (ullMax < m_ppullLRUCacheAccessCounter[(int)ullIndex][i])
		{
			ullMax = m_ppullLRUCacheAccessCounter[(int)ullIndex][i];
		}
	}
	return ullMax;
}

unsigned long long Cache::FindMaxCountOfFIFO(unsigned long long ullIndex)
{
	unsigned long long ullMax = m_ppullFIFOCounter[(int)ullIndex][0];

	// Find mru number
	for (int i = 1; i < m_iAssoc; i++)
	{

		if (ullMax < m_ppullFIFOCounter[(int)ullIndex][i])
		{
			ullMax = m_ppullFIFOCounter[(int)ullIndex][i];
		}
	}
	return ullMax;
}

bool Cache::Prefetch(unsigned long long ullIndex, unsigned long long ullTag)
{
	bool bBlockPlaced = false;
	for (int col = 0; col < m_iAssoc; ++col)
	{
		if (m_ppbValidityFlag[(int)ullIndex][col] == false)
		{
			m_ppiPrefetchTimer[(int)ullIndex][col] = 10;
			m_ppbPrefetched[(int)ullIndex][col] = true;

			// We found an invalid line. Use it!
			m_ppullTagArray[(int)ullIndex][col] = ullTag;

			// Reset its validity flag
			m_ppbValidityFlag[(int)ullIndex][col] = true;

			bBlockPlaced = true;
			// This is a cold miss
			// m_pullReadMissCounter[(int)ullIndex] ++; // TO_DO: Check this

			switch (m_iRepPolicy)
			{
			case 0:
			{
				// Find the max count
				unsigned long long ullMax = FindCountOfMRU(ullIndex);

				m_ppullLRUCacheAccessCounter[(int)ullIndex][col] = ullMax + 1;
				break;
			}
			}

			break;
		}
		else
		{
			if (m_ppullTagArray[(int)ullIndex][col] == ullTag)
			{
				// Do nothing if block is already present
				bBlockPlaced = true;

				break;
			}
		}
	}

	if (!bBlockPlaced)
	{
		switch (m_iRepPolicy)
		{
		case 0:
		{
			// LRU implementation
			int col;

			// Find the minimum counter, max count, min counter column number
			unsigned long long ullMin = m_ppullLRUCacheAccessCounter[(int)ullIndex][0];
			unsigned long long ullMax = m_ppullLRUCacheAccessCounter[(int)ullIndex][0];

			int iMinCol = 0;
			for (col = 1; col < m_iAssoc; col++)
			{
				if (ullMin > m_ppullLRUCacheAccessCounter[(int)ullIndex][col])
				{
					ullMin = m_ppullLRUCacheAccessCounter[(int)ullIndex][col];
					iMinCol = col;
				}
				if (ullMax < m_ppullLRUCacheAccessCounter[(int)ullIndex][col])
				{
					ullMax = m_ppullLRUCacheAccessCounter[(int)ullIndex][col];
				}
			}

			m_ppullTagArray[(int)ullIndex][iMinCol] = ullTag;
			m_ppiPrefetchTimer[(int)ullIndex][iMinCol] = 10;
			m_ppbPrefetched[(int)ullIndex][iMinCol] = true;
			break;
		}
		}
	}
	return true;
}

void Cache::reducePrefetchCount()
{
	for (int i = 0; i < m_iSets; ++i)
	{
		for (int j = 0; j < m_iAssoc; j++)
		{
			if (m_ppiPrefetchTimer[i][j] > 0)
				m_iPrefetchCounter = m_ppiPrefetchTimer[i][j]--;
		}
	}
}

bool Cache::Read(unsigned long long ullIndex, unsigned long long ullTag)
{
	bool bCachePlaced = false;
	bool bReturn = false; // When bReturn is true it means read miss.
	// Increment Read Counter
	m_ullReadCounter++;

	for (int col = 0; col < m_iAssoc; ++col)
	{
		if (m_ppbValidityFlag[(int)ullIndex][col] == false)
		{
			// We found an invalid line. Use it!
			m_ppullTagArray[(int)ullIndex][col] = ullTag;

			// Reset its validity flag
			m_ppbValidityFlag[(int)ullIndex][col] = true;

			// This is a cold miss
			m_pullReadMissCounter[(int)ullIndex] ++;

			// Update placed flag 
			bCachePlaced = true;

			// We need to send this tag to next level also
			bReturn = true;

			// Update the access counter - lru
			switch (m_iRepPolicy)
			{
			case 0:
			{
				// Find the max count
				unsigned long long ullMax = FindCountOfMRU(ullIndex);

				m_ppullLRUCacheAccessCounter[(int)ullIndex][col] = ullMax + 1;
				break;
			}

			case 1:
			{
				// FIFO
				// Find the max count
				unsigned long long ullMax = FindMaxCountOfFIFO(ullIndex);

				m_ppullFIFOCounter[(int)ullIndex][col] = ullMax + 1;
				break;
			}

			case 2:
			{
				// psuedo LRU
				int iArrIndex = col + (m_iAssoc - 1);
				int iTempCol = col;
				bool bValue;
				while (iArrIndex != 0)
				{
					iArrIndex = (iArrIndex - 1) / 2;
					bValue = iTempCol % 2 == 0 ? true : false;
					m_ppbPseudoLRU[(int)ullIndex][iArrIndex] = bValue;
					iTempCol = iTempCol / 2;
				}
				break;
			}
			}
			break;
		}
		else
		{
			if (m_ppullTagArray[(int)ullIndex][col] == ullTag)
			{
				// Its a hit
				m_pullReadHitCounter[(int)ullIndex] ++;

				// We dont need to send this tag to next level also
				bReturn = false;

				// Update the access counter - lru
				switch (m_iRepPolicy)
				{
				case 0:
				{
					// Find the max count
					unsigned long long ullMax = FindCountOfMRU(ullIndex);

					m_ppullLRUCacheAccessCounter[(int)ullIndex][col] = ullMax + 1;
					break;
				}

				case 1:
				{
					// When we have a hit do not do anything for FIFO
					break;
				}

				case 2:
				{
					// Psuedo LRU - hit
					int iArrIndex = col + (m_iAssoc - 1);
					int iTempCol = col;
					bool bValue;
					while (iArrIndex != 0)
					{
						iArrIndex = (iArrIndex - 1) / 2;
						bValue = iTempCol % 2 == 0 ? true : false;
						m_ppbPseudoLRU[(int)ullIndex][iArrIndex] = bValue;
						iTempCol = iTempCol / 2;
					}
					break;
				}
				}

				// Update placed flag 
				bCachePlaced = true;
				break;
			}
		}
	}

	if (bCachePlaced == false)
	{
		// We need a replacement policy
		// replaceCache();

		// Its a miss
		m_pullReadMissCounter[(int)ullIndex] ++;

		// We need to send this tag to next level also
		bReturn = true;

		switch (m_iRepPolicy)
		{
		case 0:
		{
			// LRU implementation
			int col;

			// Find the minimum counter, max count, min counter column number
			unsigned long long ullMin = m_ppullLRUCacheAccessCounter[(int)ullIndex][0];
			unsigned long long ullMax = m_ppullLRUCacheAccessCounter[(int)ullIndex][0];

			int iMinCol = 0;
			for (col = 1; col < m_iAssoc; col++)
			{
				if (ullMin > m_ppullLRUCacheAccessCounter[(int)ullIndex][col])
				{
					ullMin = m_ppullLRUCacheAccessCounter[(int)ullIndex][col];
					iMinCol = col;
				}
				if (ullMax < m_ppullLRUCacheAccessCounter[(int)ullIndex][col])
				{
					ullMax = m_ppullLRUCacheAccessCounter[(int)ullIndex][col];
				}
			}

			m_ppullTagArray[(int)ullIndex][iMinCol] = ullTag;

			if (m_ppbDirtyBit[(int)ullIndex][iMinCol] == true)
			{
				// If the dirty bit is set, then unset it because data from L2 will be used to update L1
				m_ppbDirtyBit[(int)ullIndex][iMinCol] = false;

				m_ullWriteBackCounter++;
			}
			else
			{
				m_ullCleanEvictions++;
			}

			// Update Minimum Column's tag
			m_ppullLRUCacheAccessCounter[(int)ullIndex][iMinCol] = ullMax + 1;

			break;
		}

		case 1:
		{
			// FIFO
			// Find the minimum counter, max count, min counter column number
			unsigned long long ullMin = m_ppullFIFOCounter[(int)ullIndex][0];
			unsigned long long ullMax = m_ppullFIFOCounter[(int)ullIndex][0];

			int iMinCol = 0;
			for (int col = 1; col < m_iAssoc; col++)
			{
				if (ullMin > m_ppullFIFOCounter[(int)ullIndex][col])
				{
					ullMin = m_ppullFIFOCounter[(int)ullIndex][col];
					iMinCol = col;
				}
				if (ullMax < m_ppullFIFOCounter[(int)ullIndex][col])
				{
					ullMax = m_ppullFIFOCounter[(int)ullIndex][col];
				}
			}

			m_ppullTagArray[(int)ullIndex][iMinCol] = ullTag;

			if (m_ppbDirtyBit[(int)ullIndex][iMinCol] == true)
			{
				// If the dirty bit is set, then unset it because data from L2 will be used to update L1
				m_ppbDirtyBit[(int)ullIndex][iMinCol] = false;

				m_ullWriteBackCounter++;
			}
			else
			{
				m_ullCleanEvictions++;
			}

			// Update Minimum Column's tag
			m_ppullFIFOCounter[(int)ullIndex][iMinCol] = ullMax + 1;
			break;
		}

		case 2:
		{
			// Find the pseudoLRU
			int iTemp = 0;
			int iMinCol = 0;

			while (iTemp < m_iAssoc - 1)
			{
				iTemp = m_ppbPseudoLRU[(int)ullIndex][iTemp] == false ? 2 * iTemp + 1 : 2 * iTemp + 2;
			}

			iMinCol = iTemp - (m_iAssoc - 1);

			// Update the LRU again
			int iArrIndex = iMinCol + (m_iAssoc - 1);
			int iTempCol = iMinCol;
			bool bValue;
			while (iArrIndex != 0)
			{
				iArrIndex = (iArrIndex - 1) / 2;
				bValue = iTempCol % 2 == 0 ? true : false;
				m_ppbPseudoLRU[(int)ullIndex][iArrIndex] = bValue;
				iTempCol = iTempCol / 2;
			}

			if (m_ppbDirtyBit[(int)ullIndex][iMinCol] == true)
			{
				// If the dirty bit is set, then unset it because data from L2 will be used to update L1
				m_ppbDirtyBit[(int)ullIndex][iMinCol] = false;

				m_ullWriteBackCounter++;
			}
			else
			{
				m_ullCleanEvictions++;
			}

			// Update tag
			m_ppullTagArray[(int)ullIndex][iMinCol] = ullTag;
			break;
		}
		}
	}
	return bReturn;
}

bool Cache::Write(unsigned long long ullIndex, unsigned long long ullTag)
{
	bool bCachePlaced = false;
	bool bReturn = false;
	m_ullWriteCounter++;
	
	// Find an invalid line or line which contains the tag
	for (int col = 0; col < m_iAssoc; ++col)
	{
		if (m_ppbValidityFlag[(int)ullIndex][col] == false)
		{
			// We found an invalid line. Use it!
			m_ppullTagArray[(int)ullIndex][col] = ullTag;

			// Set its validity flag
			m_ppbValidityFlag[(int)ullIndex][col] = true;

			// Set its dirty bit
			m_ppbDirtyBit[(int)ullIndex][col] = true;

			// This is a cold miss
			m_pullWriteMissCounter[(int)ullIndex] ++;

			// Update placed flag 
			bCachePlaced = true;

			// We need to send this tag to next level also
			bReturn = true;

			// Update the access counter
			switch (m_iRepPolicy)
			{
			case 0:
			{
				// Find the max count
				unsigned long long ullMax = FindCountOfMRU(ullIndex);

				m_ppullLRUCacheAccessCounter[(int)ullIndex][col] = ullMax + 1;
				break;
			}

			case 1:
			{
				// FIFO
				// Find the max count
				unsigned long long ullMax = FindMaxCountOfFIFO(ullIndex);

				m_ppullFIFOCounter[(int)ullIndex][col] = ullMax + 1;
				break;
			}

			case 2:
			{
				// Psuedo LRU - hit
				int iArrIndex = col + (m_iAssoc - 1);
				int iTempCol = col;
				bool bValue;
				while (iArrIndex != 0)
				{
					iArrIndex = (iArrIndex - 1) / 2;
					bValue = iTempCol % 2 == 0 ? true : false;
					m_ppbPseudoLRU[(int)ullIndex][iArrIndex] = bValue;
					iTempCol = iTempCol / 2;
				}
				break;
			}
			} // end of switch case
			break;
		}
		else
		{
			if (m_ppullTagArray[(int)ullIndex][col] == ullTag)
			{
				// Its a hit
				m_pullWriteHitCounter[(int)ullIndex] ++;

				// Set its dirty bit
				m_ppbDirtyBit[(int)ullIndex][col] = true;

				// Update placed flag 
				bCachePlaced = true;

				// We dont need to send this tag to next level for NINE
				bReturn = false;

				// Update the access counter
				switch (m_iRepPolicy)
				{
				case 0:
				{
					// Find the max count
					unsigned long long ullMax = FindCountOfMRU(ullIndex);

					m_ppullLRUCacheAccessCounter[(int)ullIndex][col] = ullMax + 1;
					break;
				}

				case 1:
				{
					// FIFO - Do Nothing
					break;
				}

				case 2:
				{
					// Psuedo LRU - hit
					int iArrIndex = col + (m_iAssoc - 1);
					int iTempCol = col;
					bool bValue;
					while (iArrIndex != 0)
					{
						iArrIndex = (iArrIndex - 1) / 2;
						bValue = iTempCol % 2 == 0 ? true : false;
						m_ppbPseudoLRU[(int)ullIndex][iArrIndex] = bValue;
						iTempCol = iTempCol / 2;
					}
					break;
				}
				}
				break;
			}
		}
	}

	// No space in the set? Evict a line and make space
	if (bCachePlaced == false)
	{
		// We need a replacement policy
		// replaceCache();

		// Its a miss
		m_pullWriteMissCounter[(int)ullIndex] ++;

		// We need to send this tag to next level also
		bReturn = true;

		switch (m_iRepPolicy)
		{
		case 0:
		{
			// LRU implementation
			int col;

			// Find the minimum counter, max count, min counter column number
			unsigned long long ullMin = m_ppullLRUCacheAccessCounter[(int)ullIndex][0];
			unsigned long long ullMax = m_ppullLRUCacheAccessCounter[(int)ullIndex][0];

			int iMinCol = 0;
			for (col = 1; col < m_iAssoc; col++)
			{
				if (ullMin > m_ppullLRUCacheAccessCounter[(int)ullIndex][col])
				{
					ullMin = m_ppullLRUCacheAccessCounter[(int)ullIndex][col];
					iMinCol = col;
				}
				if (ullMax < m_ppullLRUCacheAccessCounter[(int)ullIndex][col])
				{
					ullMax = m_ppullLRUCacheAccessCounter[(int)ullIndex][col];
				}
			}

			m_ppullTagArray[(int)ullIndex][iMinCol] = ullTag;

			// Update Minimum Column's tag
			m_ppullLRUCacheAccessCounter[(int)ullIndex][iMinCol] = ullMax + 1;

			if (m_ppbDirtyBit[(int)ullIndex][iMinCol] == true)
			{
				m_ullWriteBackCounter++;
			}
			else
			{
				m_ullCleanEvictions++;
			}

			// Set its dirty bit
			m_ppbDirtyBit[(int)ullIndex][iMinCol] = true;

			break;
		}

		case 1:
		{
			// FIFO
			// Find the minimum counter, max count, min counter column number
			unsigned long long ullMin = m_ppullFIFOCounter[(int)ullIndex][0];
			unsigned long long ullMax = m_ppullFIFOCounter[(int)ullIndex][0];

			int iMinCol = 0;
			for (int col = 1; col < m_iAssoc; col++)
			{
				if (ullMin > m_ppullFIFOCounter[(int)ullIndex][col])
				{
					ullMin = m_ppullFIFOCounter[(int)ullIndex][col];
					iMinCol = col;
				}
				if (ullMax < m_ppullFIFOCounter[(int)ullIndex][col])
				{
					ullMax = m_ppullFIFOCounter[(int)ullIndex][col];
				}
			}

			m_ppullTagArray[(int)ullIndex][iMinCol] = ullTag;

			// Update Minimum Column's tag
			m_ppullFIFOCounter[(int)ullIndex][iMinCol] = ullMax + 1;

			if (m_ppbDirtyBit[(int)ullIndex][iMinCol] == true)
			{
				m_ullWriteBackCounter++;
			}
			else
			{
				m_ullCleanEvictions++;
			}

			// Set its dirty bit
			m_ppbDirtyBit[(int)ullIndex][iMinCol] = true;
			break;
		}

		case 2:
		{
			// Find the pseudoLRU
			int iTemp = 0;
			int iMinCol = 0;

			while (iTemp < m_iAssoc - 1)
			{
				iTemp = m_ppbPseudoLRU[(int)ullIndex][iTemp] == false ? 2 * iTemp + 1 : 2 * iTemp + 2;
			}

			iMinCol = iTemp - (m_iAssoc - 1);

			int iArrIndex = iMinCol + (m_iAssoc - 1);
			int iTempCol = iMinCol;
			bool bValue;
			while (iArrIndex != 0)
			{
				iArrIndex = (iArrIndex - 1) / 2;
				bValue = iTempCol % 2 == 0 ? true : false;
				m_ppbPseudoLRU[(int)ullIndex][iArrIndex] = bValue;
				iTempCol = iTempCol / 2;
			}

			// Update tag
			m_ppullTagArray[(int)ullIndex][iMinCol] = ullTag;

			if (m_ppbDirtyBit[(int)ullIndex][iMinCol] == true)
			{
				m_ullWriteBackCounter++;
			}
			else
			{
				m_ullCleanEvictions++;
			}

			// Set its dirty bit
			m_ppbDirtyBit[(int)ullIndex][iMinCol] = true;
			break;
		}
		}
	}
	return bReturn;
}

//int main(int argc, char* argv[])
//{
//	int iBlockSize, iL1Size, iL1Assoc, iL2Size, iL2Assoc, iRepPolicy, iIncPolicy;
//	string fileName;
//	string strRepPolicy, strIncPolicy;
//
//	// Parse command line arguments
//	if (argc > 1)
//	{
//		char *endptr;
//		int base = 10;
//		iBlockSize = strtol(argv[1], &endptr, base);
//		iL1Size = strtol(argv[2], &endptr, base);
//		iL1Assoc = strtol(argv[3], &endptr, base);
//		iL2Size = strtol(argv[4], &endptr, base);
//		iL2Assoc = strtol(argv[5], &endptr, base);
//		iRepPolicy = strtol(argv[6], &endptr, base);
//		iIncPolicy = strtol(argv[7], &endptr, base);
//		fileName = argv[8];
//	}
//	else
//	{
//		return 1;
//	}
//
//	switch (iRepPolicy)
//	{
//	case 0:
//	{
//		strRepPolicy = "LRU";
//		break;
//	}
//	case 1:
//	{
//		strRepPolicy = "FIFO";
//		break;
//	}
//	case 2:
//	{
//		strRepPolicy = "Pseudo";
//		break;
//	}
//	case 3:
//	{
//		strRepPolicy = "Optimal";
//		break;
//	}
//	}
//
//	switch (iIncPolicy)
//	{
//	case 0:
//	{
//		strIncPolicy = "non-inclusive";
//		break;
//	}
//	case 1:
//	{
//		strIncPolicy = "inclusive";
//		break;
//	}
//	case 2:
//	{
//		strIncPolicy = "exclusive";
//		break;
//	}
//	}
//
//	Cache objL1Cache(1, iBlockSize, iL1Size, iL1Assoc, iRepPolicy, iIncPolicy);
//	objL1Cache.Init();
//
//	Cache objL2Cache(2, iBlockSize, iL2Size, iL2Assoc, iRepPolicy, iIncPolicy);
//	if (iL2Size != 0 && iL2Assoc !=0 )
//		objL2Cache.Init();
//
//	// Print the output on the validation file
//	ofstream ofs;
//	ofs.open("my_validation.txt", std::ofstream::out);
//
//	ofs << "===== Simulator configuration =====\n";
//	ofs << "BLOCKSIZE: \t \t"	<< objL1Cache.m_iBlockSize<<"\n";
//	ofs << "L1_SIZE:\t \t"		<< objL1Cache.m_iSize << "\n";
//	ofs << "L1_ASSOC:\t \t"	<< objL1Cache.m_iAssoc << "\n";
//	ofs << "L2_SIZE:\t \t" << objL2Cache.m_iSize << "\n";
//	ofs << "L2_ASSOC:\t \t" << objL2Cache.m_iAssoc << "\n";
//	ofs << "REPLACEMENT POLICY:\t" << strRepPolicy << "\n";
//	ofs << "INCLUSION PROPERTY:\t" << strIncPolicy << "\n";
//	ofs << "trace_file:\t" << fileName << "\n";
//	//ofs << "Sets:\t \t \t"		<< objL1Cache.m_iSets << "\n";//TO_DO: Remove
//	//ofs << "Index Width:\t \t" << objL1Cache.m_ullIndexWidth << "\n";//TO_DO: Remove
//	//ofs << "Tag Width:\t \t"	<< objL1Cache.m_ullTagWidth << "\n";//TO_DO: Remove
//
//	//
//	unsigned long long ullL2Reads = 0;
//
//	// Parse trace file
//	ifstream trace (fileName.c_str());
//	
//	string traceLine;
//	
//	getline(trace, traceLine);
//
//	while (!traceLine.empty()) 
//	{
//		//std::ofs << dLineCounter << " " << traceLine << '\n';
//
//		// Let's assume input will be well-formed and not validate it
//		char cOperationType = traceLine.at(0);
//
//		// Call read if operation type is read
//		if ('r' == cOperationType)
//		{
//			// Extract index and tag from address
//			string strAddress = traceLine.substr(2); 
//			unsigned long long ullAddress = strtoull(strAddress.c_str(), NULL, 16);
//			unsigned long long ullIndex;
//			
//			unsigned long long ullMask;
//			ullMask = 0xffffffffffffffff;
//			ullMask = ullMask >> (64 - objL1Cache.m_ullOffsetWidth - objL1Cache.m_ullIndexWidth);
//			ullIndex = ullAddress;
//			ullIndex = ullIndex & ullMask;
//
//			ullIndex = ullIndex >> (objL1Cache.m_ullOffsetWidth);
//			
//			unsigned long long ullTag = ullAddress >> objL1Cache.m_ullOffsetWidth + objL1Cache.m_ullIndexWidth;
//
//			// Flag to indicate if replacement is needed
//			bool bCachePlaced = false;
//			unsigned long long ullPrevWritebacks = objL1Cache.m_ullWriteBackCounter;
//
//			// Update the cache using Read()
//			bCachePlaced = objL1Cache.Read(ullIndex, ullTag);
//			if (bCachePlaced)
//			{
//				if (iL2Assoc != 0 || iL2Size != 0)
//				{
//					ullL2Reads++;
//					objL2Cache.Read(ullIndex, ullTag);
//				}
//			}
//
//			if (ullPrevWritebacks != objL1Cache.m_ullWriteBackCounter)
//			{
//				if (iL2Assoc != 0 || iL2Size != 0)
//					objL2Cache.Write(ullIndex, ullTag);
//			}
//		}
//
//		// Call write if operaion type is write
//		else if ('w' == cOperationType)
//		{
//			// Extract index and tag from address
//			string strAddress = traceLine.substr(2);
//			//ofs << "Address: " << objAddress.strAddress << "\n";
//
//			unsigned long long ullAddress = strtoull(strAddress.c_str(), NULL, 16);
//			unsigned long long ullIndex;
//			unsigned long long ullMask;
//			ullMask = 0xffffffffffffffff;
//			ullMask = ullMask >> (64 - objL1Cache.m_ullOffsetWidth - objL1Cache.m_ullIndexWidth);
//			ullIndex = ullAddress;
//			ullIndex = ullIndex & ullMask;
//
//			ullIndex = ullIndex >> (objL1Cache.m_ullOffsetWidth);
//
//			unsigned long long ullTag = ullAddress >> objL1Cache.m_ullOffsetWidth + objL1Cache.m_ullIndexWidth;
//
//			// Flag to indicate if replacement is needed
//			bool bCachePlaced = false;
//			unsigned long long ullPrevWritebacks = objL1Cache.m_ullWriteBackCounter;
//			// Use index and tag as input to write function
//			bCachePlaced = objL1Cache.Write(ullIndex, ullTag);
//			if (bCachePlaced)
//			{
//				if (iL2Assoc != 0 || iL2Size != 0)
//				{
//					ullL2Reads++;
//					objL2Cache.Read(ullIndex, ullTag);
//				}
//			}
//
//			if (ullPrevWritebacks != objL1Cache.m_ullWriteBackCounter)
//			{
//				if (iL2Assoc != 0 || iL2Size != 0)
//					objL2Cache.Write(ullIndex, ullTag);
//			}
//		}
//		else
//		{
//			// Invalid input
//
//		}
//		//trace.getline(cTraceArr, 32);
//		getline(trace, traceLine);
//	}
//
//	trace.close();
//	
//	ofs << "===== Simulation results (raw) =====\n";
//	ofs << "a. number of L1 reads: \t\t" << objL1Cache.m_ullReadCounter << endl;
//	ofs << "b. number of L1 read misses: \t" << objL1Cache.calculateReadMisses() << endl;
//	ofs << "c. number of L1 writes:  \t" << objL1Cache.m_ullWriteCounter << endl;
//	ofs << "d. number of L1 write misses: \t" << objL1Cache.calculateWriteMisses() << endl;
//	ofs << "e. L1 miss rate: \t\t" << objL1Cache.calculateMissRate() << endl;
//	ofs << "f. number of L1 writebacks: \t" << objL1Cache.m_ullWriteBackCounter << endl;
//
//	if (iL2Assoc == 0 && iL2Size == 0)
//	{
//		ofs << "g. number of L2 reads: \t\t" << 0 << endl;
//		ofs << "h. number of L2 read misses: \t" << 0 << endl;
//		//ofs << "i. number of L2 read hits: \t" << objL2Cache.calculateReadHits() << endl;
//		ofs << "i. number of L2 writes: \t" << 0 << endl;
//		ofs << "j. number of L2 write misses: \t" << 0 << endl;
//		//ofs << "number of L2 write hits: \t" << objL2Cache.calculateWriteHits() << endl;
//		ofs << "k. L2 miss rate: \t\t" << 0 << endl;
//		ofs << "l. number of L2 writebacks: \t" << 0 << endl;
//		ofs << "m. total memory traffic: \t" << objL1Cache.calculateReadMisses() + objL1Cache.calculateWriteMisses() + objL1Cache.m_ullWriteBackCounter;
//	}
//	else
//	{
//		ofs << "g. number of L2 reads: \t\t" << objL1Cache.calculateReadMisses() + objL1Cache.calculateWriteMisses() << endl;
//		ofs << "h. number of L2 read misses: \t" << objL2Cache.calculateReadMisses() << endl;
//		if (iIncPolicy == 0 || iIncPolicy == 1)
//			ofs << "i. number of L2 writes: \t" << objL1Cache.m_ullWriteBackCounter << endl;
//		else
//			ofs << "i. number of L2 writes: \t" << objL1Cache.m_ullWriteBackCounter + objL1Cache.m_ullCleanEvictions << endl;
//		ofs << "j. number of L2 write misses: \t" << objL2Cache.calculateWriteMisses() << endl;
//		ofs << "k. L2 miss rate: \t\t" << objL2Cache.calculateMissRate() << endl;
//		ofs << "l. number of L2 writebacks: \t" << objL2Cache.m_ullWriteBackCounter << endl;
//		ofs << "m. total memory traffic: \t" << objL1Cache.calculateReadMisses() + objL1Cache.calculateWriteMisses() + objL1Cache.m_ullWriteBackCounter;
//	}
//
//	ofs.close();
//	return 0;
//}