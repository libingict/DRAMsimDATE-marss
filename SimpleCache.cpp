#include <iostream>

#include "SimpleCache.h"
#include "Transaction.h"
//#include "Simulator.h"
//#include "SystemConfiguration.h"
using std::cout;
using std::endl;

using DRAMSim::Transaction;
using DRAMSim::WarmupCycle;

BlSim::CacheBlock* BlSim::CacheBlock::GetWriteBackBlock() {
    // 鍙湁drity,骞朵笖鏈夋暟鎹殑鏃跺�鎵嶉渶瑕佷細鍐欏埌鍐呭瓨
    if (m_dirty == 0 || m_block_addr == INVALID_BLOCK)
        return NULL;
    return new CacheBlock(m_block_size, m_block_addr, m_block_tag);
}

void BlSim::CacheBlock::Refresh(const CacheAddress& cache_addr) {
    m_block_addr = cache_addr.addr;
    m_block_tag = cache_addr.tag;
    m_dirty = 0;
    m_next = NULL;
    m_prev = NULL;
}

void BlSim::CacheBlock::Access(uint32_t memop) {
	if(memop == MEM_WRITE) {
		m_dirty = 1;
	}
}

BlSim::CacheSet::CacheSet(uint32_t way_count, uint32_t block_size):
	m_way_count(way_count) {
	CacheBlock* p_block = NULL;
	CacheBlock* p_next_block = NULL;
	p_block = new CacheBlock(block_size);
	m_p_mru_block = p_block;

	for (int i = 1; i < way_count; i++) {
		p_next_block = new CacheBlock(block_size);
		p_block->m_next = p_next_block;
		p_next_block->m_prev = p_block;
		p_block = p_next_block;		
	}
	m_p_lru_block = p_next_block;
}

BlSim::CacheSet::~CacheSet() {
	CacheBlock* p_block = NULL;
	CacheBlock* p_next_block = NULL;

	p_block = m_p_mru_block;
	while (p_block) {
		p_next_block = p_block->m_next;
		delete p_block;
		p_block = p_next_block;
	}
}

/*
 * 鏍规嵁tag鎵惧埌block
 */
bool BlSim::CacheSet::FindToDo(int64_t tag) {
	CacheBlock *p_block = m_p_mru_block;
	while (p_block) {
		if (tag == p_block->m_block_tag) {
		    // 濡傛灉涓嶆槸mru鍧�
            if (p_block->m_prev) {
                p_block->m_prev->m_next = p_block->m_next;
                if (p_block->m_next) {
                    p_block->m_next->m_prev = p_block->m_prev;
                } else {
                    // 鏄痩ru鍧楋紝闇�鏇存柊lru鍧椾负prev
                    m_p_lru_block = p_block->m_prev;
                }
                // 鏇存柊mru鍧楋紝璁惧綋鍓嶅潡涓簃ru鍧�
                p_block->m_next = m_p_mru_block;
                m_p_mru_block->m_prev = p_block;
                p_block->m_prev = NULL;
                m_p_mru_block = p_block;
            }
            return true;
		}
		p_block = p_block->m_next;
	}
	return false;
}

/*
 * 鍏坋vict鐨刲ru鍧楋紝鍐嶈浇鍏ユ柊鐨勫潡锛屽苟璁剧疆涓簃ru鍧楋紝杩斿洖闇�鍐欏洖鐨勫潡
 */
BlSim::CacheBlock* BlSim::CacheSet::LoadNewBlock(const CacheAddress& cache_addr) {
    // evict lru鍧�
    CacheBlock* p_block = m_p_lru_block;
    m_p_lru_block = m_p_lru_block->m_prev;
    m_p_lru_block->m_next = NULL;
    // 鎶妉ru鍧楀啓鍥炲埌鍐呭瓨
    CacheBlock* block = p_block->GetWriteBackBlock();
    p_block->Refresh(cache_addr);
    // 璁剧疆褰撳墠鍧椾负mru鍧�
    p_block->m_next = m_p_mru_block;
    m_p_mru_block->m_prev = p_block;
    m_p_mru_block = p_block;
    return block;
}

BlSim::Cache::Cache(uint32_t cores,
                    MemorySystem* memory_system,
                    TransactionReceiver* receiver) :
    m_memory_system(memory_system),
    m_transaction_receiver(receiver) {
    m_cache_capacity = (32UL << 20) * cores;
    m_cache_way_count = 8;
    m_block_size = 64;
    m_cache_set_capacity = m_block_size * m_cache_way_count;
    m_cache_set_count = m_cache_capacity / m_cache_set_capacity;

    m_block_low_bits = FloorLog2(m_block_size);
    m_set_index_bits = FloorLog2(m_cache_set_count);

    m_block_low_mask = (1UL << m_block_low_bits) - 1;
    m_set_index_mask = (1UL << m_set_index_bits) - 1;

    m_hit_count = 0;
    m_writeback_count = 0;
    m_total_count = 0;

    m_cache_sets = new CacheSet*[m_cache_set_count];
    for (int i = 0; i < m_cache_set_count; ++i) {
        m_cache_sets[i] = new CacheSet(m_cache_way_count, m_block_size);
    }
}

BlSim::Cache::~Cache() {
    for (int i = 0; i < m_cache_set_count; ++i) {
        if (m_cache_sets[i]) {
            delete m_cache_sets[i];
            m_cache_sets[i] = NULL;
        }
    }
    delete [] m_cache_sets;
}

/*
 * 璁＄畻缁勭储寮曞拰tag
 * 鍏堝彸绉诲潡浣嶏紝寰楀埌鍧楀湴鍧�紝鍐嶅彇妯＄粍鏁帮紝鑾峰彇缁勭储寮�
 * 鍐嶅彸绉荤粍浣嶏紝寰楀埌tag
 */
BlSim::CacheAddress BlSim::Cache::GetCacheAddress(uint64_t maddr) {
	uint64_t tmp = maddr;
	tmp >>= m_block_low_bits;
	uint32_t index = tmp & m_set_index_mask;

	tmp >>= m_set_index_bits;
	int64_t tag = tmp;
	CacheAddress addr(maddr, index, tag);
	return addr;
}

void BlSim::Cache::WriteBack(CacheBlock* block, uint64_t clock_cycle) {
    Transaction* trans = new Transaction(Transaction::DATA_WRITE,
                                         block->m_block_addr,
                                         NULL,
                                         block->m_block_size,
                                         clock_cycle);
    //cout << "write back. addr: " << block->m_block_addr
    //     << "\tcycle: " << clock_cycle << endl;
    if(clock_cycle > WarmupCycle){
    	m_writeback_count++;
    }
    if (m_memory_system->addTransaction(trans))
	m_transaction_receiver->addPending(trans, clock_cycle);
}

bool BlSim::Cache::Access(uint64_t maddr, uint32_t memop,
                          uint64_t clock_cycle) {
    //cout << "access addr:" << maddr << "\top:" << memop << endl;
	if(clock_cycle > WarmupCycle){
		m_total_count++;
	}
    // 鑾峰彇缁勫亸绉诲拰tag
    CacheAddress cache_addr = GetCacheAddress(maddr);
    // 鏍规嵁tag鍦ㄧ浉搴旂粍涓煡鎵綽lock
    bool is_hit = m_cache_sets[cache_addr.index]->FindToDo(cache_addr.tag);
    if (is_hit) {
    	if(clock_cycle>WarmupCycle){
            m_hit_count++;
    	}
    } else {
        // 娌℃湁鍛戒腑鍒欓渶瑕佽浇鍏ache锛屽苟evict lru鍧�
        CacheBlock* block = m_cache_sets[cache_addr.index]->LoadNewBlock(cache_addr);
        if (block) {
            WriteBack(block, clock_cycle);
            delete block;
        }
    }
    // 璁块棶褰撳墠鍧�
    m_cache_sets[cache_addr.index]->GetMruBlock()->Access(memop);
    return is_hit;
}

void BlSim::Cache::DumpStatistic() {
    float hit_rate = (float)(m_hit_count) / m_total_count;
    cout << "total: " << m_total_count
         << "\twrite back: " << m_writeback_count
         << "\thit: " << m_hit_count
         << "\thit rate: " << hit_rate << endl;
}

