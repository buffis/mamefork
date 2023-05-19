#ifndef MAME_CPU_SH_SH3_CACHE_H
#define MAME_CPU_SH_SH3_CACHE_H

#pragma once

// Simulates the SH3 cache in terms of external bus cycles.
//
// Information about the cache from SH7709S Datasheet (Section 5):
// - Capacity 16 kbytes
// - Structure Instruction/data mixed, 4-way set associative
// - Line size 16 bytes
// - Number of entries 256 entries/way
// - Replacement method Least-recently-used (LRU) algorithm
//
// The following features are not implemented:
// - Actual caching of data. This only simulates cache hits/misses.
// - MMU (Always disabled for cv1k games).
// - Locking of ways (Through CCR2)

struct sh3_cache_entry {
    bool valid;
    bool written_write_back;
    u32 address; // Only 22 lower bits are used.
    //u32 data[4]; // LW0-LW3: Longword data 0-3
};

enum CACHE_TYPE
{
    DISABLED,
    WRITE_BACK,
    WRITE_THROUGH
};

class sh3_cache 
{
    public:
    sh3_cache()
    {
        setup_lru_lookup_mask();
    }

    inline u8 get_oldest_way(u8 entry)
    {
        return m_lru_lookup_mask[m_lru[entry]];
    }

    inline void update_lru(u8 entry, u8 used_way)
    {
        m_lru[entry] = (m_lru[entry] & m_lru_and_masks[used_way]) | m_lru_or_masks[used_way];
    }

    // Gets the number of wait states to insert for a memory access to a memory Area.
    // Uses hardcoded values that CV1K games use:
    // WCR2 = 0xFDD7 '1111 1101 1101 0111'
    //   A0W = 0b111
    //   A2W = 0b010
    //   A3W = 0b010
    //   A4W = 0b011
    //   A5W = 0b111
    //   A6W = 0b111
    //
    // Separately to this, a wait state will also need to be inserted whenever switching Area
    // or switching between read/write.
    // WCR1 = 0x9551 '1001 0101 0101 0001'
    //   WAITSEL = 1
    //   A0IW, A2IW, A3IW, A4IW, A5IW, A6IW = 1   
    inline int area_wait_states(int area)
    {
        switch (area)
        {
            case 0: return 10;
            // There is no Area 1.
            case 2: return 2;
            case 3: return 2;
            case 4: return 3;
            case 5: return 10;
            case 6: return 10;
            default: return 0;
        }
    }

    inline int get_area(offs_t A)
    {
        if (A >= 0x1C000000) return -1; // Reserved area.
        if (A >= 0x18000000) return 6;
        if (A >= 0x14000000) return 5;
        if (A >= 0x10000000) return 4;
        if (A >= 0x0c000000) return 3;
        if (A >= 0x08000000) return 2;
        if (A >= 0x04000000) return -1; // Internal I/O.
        return 0;
    }

    // Currently CCR is not respected for CB of WR.
    // It is assumed that CCR is set to 0x00000009 which is what all CV1K games seem to use.
    // This means:
    // CB = 0: P1 address space is write-through.
    // WT = 0: P0, U0 and P3 address space are write-back.
    CACHE_TYPE get_cache_type(offs_t A)
    {
        A = (A >> 24) & 0xff; // Only top byte is relevant.
        if (A < 0x80) // P0 address space.
            return CACHE_TYPE::WRITE_BACK;
        if (A < 0xA0) // P1 address space.
            return CACHE_TYPE::WRITE_THROUGH;
        if (A < 0xC0) // P2 address space.
            return CACHE_TYPE::DISABLED;
        if (A < 0xE0) // P3 address space.
            return CACHE_TYPE::WRITE_BACK;
        else // P4 address space.
            return CACHE_TYPE::DISABLED;
    }


    int consume_wait_states(int available_cycles)
    {
        if (m_wait_states_inserted == 0)
            return available_cycles;

        if (available_cycles >= m_wait_states_inserted*2)
        {
//            printf("Ate %d wait states\n", m_wait_states_inserted); 

            available_cycles -= m_wait_states_inserted*2;
            m_wait_states_inserted = 0;
            return available_cycles;
        }
        else
        {
//            printf("Ate %d wait states\n", available_cycles);
            m_wait_states_inserted -= (available_cycles / 2);
            return 0;
        }
    }

    // Returns true on cache hit.
    bool read(offs_t A)
    {
        m_reads++;

        int area = get_area(A);

        if (get_cache_type(A) == CACHE_TYPE::DISABLED)
        {
            m_wait_states_inserted += area_wait_states(area);
            m_disabled++;
            return false;
        }
        
        // Insert wait state if switching area, or switching between read/write.
        // Assuming WCR1 = 0x9551 (it is for CV1K).
        if (!m_last_op_was_read)
            m_wait_states_inserted++;
        else if (area != m_last_area_used)
            m_wait_states_inserted++; 
        
        m_last_area_used = area;
        m_last_op_was_read = true;
            
        // This assumes that Virtual and Logic addresses are the same (MMU is not used).
        // This is true for all CV1K games, but doesn't have to be true for other SH3 hardware.
        offs_t search_addr = (A >> 10) & 0x3FFFFF;
        u8 entry = ((A >> 4) & 0xFF);
        for (u8 way = 0; way < 4; way++) {
            if (m_lines[4 * entry + way].address == search_addr && m_lines[4 * entry + way].valid)
            {
                m_cache_hits++;
                update_lru(entry, way);
                return true;
            }
        }

        m_cache_misses++;
        m_wait_states_inserted += area_wait_states(area);
        
        u8 way = get_oldest_way(entry);
        update_lru(entry, way);
        m_lines[4 * entry + way].valid = true;
        m_lines[4 * entry + way].written_write_back = false;
        m_lines[4 * entry + way].address = search_addr;
        return false;
    }

    bool write(offs_t A)
    {
        int area = get_area(A);

        CACHE_TYPE cache_type = get_cache_type(A); 
        if (cache_type == CACHE_TYPE::DISABLED)
        {
            m_wait_states_inserted += area_wait_states(area);
            m_disabled++;
            return false;
        }

        // Insert wait state if switching area, or switching between read/write.
        // Assuming WCR1 = 0x9551 (it is for CV1K).
        if (m_last_op_was_read)
            m_wait_states_inserted++;
        else if (area != m_last_area_used)
            m_wait_states_inserted++; 
        
        m_last_area_used = area;
        m_last_op_was_read = false;

        // This assumes that Virtual and Logic addresses are the same (MMU is not used).
        // This is true for all CV1K games, but doesn't have to be true for other SH3 hardware.
        offs_t search_addr = (A >> 10) & 0x3FFFFF;
        u8 entry = ((A >> 4) & 0xFF);
        for (u8 way = 0; way < 4; way++) {
            if (m_lines[4 * entry + way].address == search_addr && m_lines[4 * entry + way].valid)
            {
                m_cache_hits++;
                update_lru(entry, way);
                m_lines[4 * entry + way].written_write_back = (cache_type == CACHE_TYPE::WRITE_BACK);
                if (cache_type == CACHE_TYPE::WRITE_THROUGH)
                    m_wait_states_inserted += area_wait_states(area);
                return true;
            }
        }

        m_cache_misses++;
        m_wait_states_inserted += area_wait_states(area);

        // In the write-through mode, no write to cache occurs in a write miss.
        // The write is only to the external memory.
        // In write-back mode, the cache is updated, and the older values is flushed to memory.
        if (cache_type == CACHE_TYPE::WRITE_BACK)
        {
            u8 way = get_oldest_way(entry);
            update_lru(entry, way);
            if (m_lines[4 * entry + way].valid && m_lines[4 * entry + way].written_write_back)
            {
                // Need to write through the data to physical memory.
                // On the actual hardware, this is first transferred to a write-back buffer
                // but in this simulation, its treated as a "regular" write. 
                int old_area = get_area(m_lines[4 * entry + way].address);
                m_wait_states_inserted += area_wait_states(old_area);
            }
            m_lines[4 * entry + way].valid = true;
            m_lines[4 * entry + way].written_write_back = true;
            m_lines[4 * entry + way].address = search_addr;
        }
        return false;
    }

    void flush()
    {
        // This is debug code.
        int num_valid = 0;
        for (int i = 0; i < 1024; i++)
        {
            if (m_lines[i].valid)
                num_valid++;
        }
        printf("Flushing cache with %d valid lines. Misses: %d, hits: %d, disabled: %d, read: %d \n", num_valid, m_cache_misses, m_cache_hits, m_disabled, m_reads);

        for (int i = 0; i < 1024; i++)
        {
            // Check if this is write-back data that needs to be flushed;
            if (m_lines[i].valid && m_lines[i].written_write_back)
            {
                int area = get_area(m_lines[i].address);
                m_wait_states_inserted += area_wait_states(area);
            }

            m_lines[i].valid = false;
            m_lines[i].written_write_back = false;
        }

        m_cache_misses = 0;
        m_cache_hits = 0;
        m_disabled = 0;
        m_reads = 0;
    }

    void set_enabled(bool enabled) { m_enabled = enabled; }

    private:
    bool m_enabled = false;
    int m_reads; // TODO: Remove
    int m_cache_misses = 0;
    int m_cache_hits = 0;
    int m_last_op_was_read;
    int m_last_area_used;
    int m_disabled;
    int m_wait_states_inserted = 0;

    // Cache consists of 4 "ways" of 256 "entries" each.
    // Lowest four values are the 4 "ways" of "entry" 0, and so on.
    sh3_cache_entry m_lines[1024];

    // From datasheet:
    // LRU (5–0)                                        Way to be Replaced
    // 000000, 000100, 010100, 100000, 110000, 110100   3
    // 000001, 000011, 001011, 100001, 101001, 101011   2
    // 000110, 000111, 001111, 010110, 011110, 011111   1
    // 111000, 111001, 111011, 111100, 111110, 111111   0
    void setup_lru_lookup_mask()
    {
        m_lru_lookup_mask[0b000000] = 3;
        m_lru_lookup_mask[0b000100] = 3;
        m_lru_lookup_mask[0b010100] = 3;
        m_lru_lookup_mask[0b100000] = 3;
        m_lru_lookup_mask[0b110000] = 3;
        m_lru_lookup_mask[0b110100] = 3;

        m_lru_lookup_mask[0b000001] = 2;
        m_lru_lookup_mask[0b000011] = 2;
        m_lru_lookup_mask[0b001011] = 2;
        m_lru_lookup_mask[0b100001] = 2;
        m_lru_lookup_mask[0b101001] = 2;
        m_lru_lookup_mask[0b101011] = 2;

        m_lru_lookup_mask[0b000110] = 1;
        m_lru_lookup_mask[0b000111] = 1;
        m_lru_lookup_mask[0b001111] = 1;
        m_lru_lookup_mask[0b010110] = 1;
        m_lru_lookup_mask[0b011110] = 1;
        m_lru_lookup_mask[0b011111] = 1;
        
        m_lru_lookup_mask[0b111000] = 0;
        m_lru_lookup_mask[0b111001] = 0;
        m_lru_lookup_mask[0b111011] = 0;
        m_lru_lookup_mask[0b111100] = 0;
        m_lru_lookup_mask[0b111110] = 0;
        m_lru_lookup_mask[0b111111] = 0;
    }

    // Actually 6 bits per entry.
    //
    // Bit 0 = way 3 newer than 2
    // Bit 1 = way 3 newer than 1
    // Bit 2 = way 2 newer than 1
    // Bit 3 = way 3 newer than 0
    // Bit 4 = way 2 newer than 0
    // Bit 5 = way 1 newer than 0
    u8 m_lru[256];

    // Holds a map of current LRU value -> way that should be replaced next.
    // See setup_lru_lookup_mask().
    u8 m_lru_lookup_mask[64];

    // Bitmasks used to modify the LRU when using a way.
    // new_lru = old_lru & m_lru_and_masks[way] | m_lru_or_masks[way]  
    u8 m_lru_and_masks[4] = {0b000111, 0b111001, 0b111110, 0b111111};
    u8 m_lru_or_masks[4]  = {0b000000, 0b100000, 0b010100, 0b001011};
};

#endif // MAME_CPU_SH_SH3_CACHE_H
