#ifndef SIMPLE_CACHE_H_
#define SIMPLE_CACHE_H_

#include <stdint.h>
//#include <stdlib.h>
#include "MemorySystem.h"
#include "Transaction.h"

#ifndef DEBUG_CACHE_SIMULATOR
#define DEBUG_CACHE_SIMULATOR
#endif

#ifndef INVALID_BLOCK
#define INVALID_BLOCK (~(0UL))
#endif
#ifndef INVALID_SUB_BLOCK_DIS
    #define INVALID_SUB_BLOCK_DIS 65
#endif
namespace BlSim {

     using DRAMSim::MemorySystem;
     using DRAMSim::TransactionReceiver;
    /*!
     *  @brief Computes floor(log2(n))
     *  Works by finding position of MSB set.
     *  @returns -1 if n == 0.
     */
    static inline uint32_t FloorLog2(uint32_t n)
    {
        uint32_t p = 0;

        if (n == 0) return 0;

        if (n & 0xffff0000) { p += 16; n >>= 16; }
        if (n & 0x0000ff00) { p +=  8; n >>=  8; }
        if (n & 0x000000f0) { p +=  4; n >>=  4; }
        if (n & 0x0000000c) { p +=  2; n >>=  2; }
        if (n & 0x00000002) { p +=  1; }

        return p;
    }
    enum Type {
        MEM_WRITE = 0,
        MEM_READ
    };
    /*

       class Transaction {
       public:
       Transaction(uint64_t addr, Type type) : m_addr(addr), m_type(type) {}
       uint64_t GetAddress() const {
       return m_addr; 
       }

       int GetType() const {
       return m_type;
       }
       private:
       uint64_t m_addr;
       Type m_type;
       };*/
    struct CacheAddress {
        uint64_t addr;
        uint32_t index;
        int64_t tag;
        CacheAddress(uint64_t maddr, uint32_t set_index, int64_t mtag) :
            addr(maddr), index(set_index), tag(mtag) {}
    };


    class CacheBlock {
        public:
            uint32_t m_block_size; //it is usual 64B
            uint64_t m_block_addr; //it is full addr, do not filter for it
            int64_t m_block_tag;  //filter the inner-set addr and the set index
            uint32_t m_dirty; //to mark if this block is written
            CacheBlock* m_next;
            CacheBlock* m_prev;

        public:
            CacheBlock(uint32_t block_size) : m_block_size(block_size),
                                              m_block_addr(INVALID_BLOCK),
                                              m_block_tag(-1),
                                              m_next(NULL),
                                              m_prev(NULL),
                                              m_dirty(0) {}
            
            CacheBlock(uint32_t size, int64_t addr, int64_t tag) :
               m_block_size(size),
               m_block_addr(addr),
               m_block_tag(tag),
               m_next(NULL),
               m_prev(NULL),
               m_dirty(0) {}

            CacheBlock* GetWriteBackBlock();
            void Refresh(const CacheAddress& cache_addr);
            void Access(uint32_t memop);
    };

    class CacheSet {

        protected:
        uint32_t m_way_count;  //the cache associaticity
        CacheBlock *m_p_mru_block;
        CacheBlock *m_p_lru_block;

        public:
        CacheSet(uint32_t way_count, uint32_t block_size);
        ~CacheSet();

        bool FindToDo(int64_t tag);

        CacheBlock* LoadNewBlock(const CacheAddress& cache_addr);

        CacheBlock* GetMruBlock() {
            return m_p_mru_block;
        }

        CacheBlock* GetLruBlock() {
            return m_p_lru_block;
        }

        void print_cache_set();
    };

    class Cache {
        protected:
            uint64_t m_cache_capacity;
            uint32_t m_cache_way_count;
            uint32_t m_block_size;

            uint32_t m_cache_set_capacity; //the size of each set
            uint32_t m_cache_set_count; //the count of cache set at each level cache

            uint32_t m_block_low_bits;  //the real low addr, 
            uint32_t m_set_index_bits;  //the cache block bits

            uint32_t m_block_low_mask;
            uint32_t m_set_index_mask;

            uint64_t m_hit_count;
            uint64_t m_writeback_count;
            uint64_t m_total_count;

            CacheSet **m_cache_sets;

            MemorySystem* m_memory_system;
  	    TransactionReceiver* m_transaction_receiver;

            CacheAddress GetCacheAddress(uint64_t maddr);

            void WriteBack(CacheBlock* block, uint64_t clock_cycle);

        public:
            Cache(uint32_t cores,
                  MemorySystem* memory_system,
                  TransactionReceiver* recevier);
            ~Cache();

            bool Access(uint64_t maddr, uint32_t memop, uint64_t clock_cycle);
            void DumpStatistic();
    };

}
#endif
