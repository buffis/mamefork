#ifndef MAME_CPU_SH_SH3_CACHE_H
#define MAME_CPU_SH_SH3_CACHE_H

#pragma once

class sh3_cache 
{
    public:
    sh3_cache() : m_wait_states(0) {}

    void set_memory(address_space* cached_mem)
    {
        m_cached_mem = cached_mem;
    }

    uint8_t read_byte(offs_t A) 
    {
        return m_cached_mem->read_byte(A);
    }

    uint16_t read_word(offs_t A)
    {
        return m_cached_mem->read_word(A);
    }

    uint32_t read_dword(offs_t A)
    {
        return m_cached_mem->read_dword(A);
    }

    void write_byte(offs_t A, uint8_t V)
    {
        m_cached_mem->write_byte(A, V);
    }

    void write_word(offs_t A, uint16_t V)
    {
        m_cached_mem->write_word(A, V);
    }

    void write_dword(offs_t A, uint32_t V)
    {
        m_cached_mem->write_dword(A, V);
    }

    private:
    address_space* m_cached_mem;
    int m_wait_states;
};

#endif // MAME_CPU_SH_SH3_CACHE_H
