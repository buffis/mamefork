// license:BSD-3-Clause
// copyright-holders:

/*
Hai Wei
WITSVS

Main components:
VRenderZERO+ MagicEyes EISC SoC
14.31818 MHz XTAL (Y1, near VR0+)
28.63636 MHz XTAL (OSC1, also near VR0+)
1.8432 MHz XTAL (X101, near MAX232)
3x Samsung K4S641632K-UC60 SDRAM
RTL8019AS Realtek Full-Duplex Ethernet Controller with Plug and Play Function
FB2022 Transmit / Receiver Filter
MAX232 Dual Driver / Receiver
battery
various jumpers
bank of 8 switches
reset push-button

System's promotional flyer claims:
- Supports touchscreen, mouse, keyboard and other input driver
- Supports IEEE802.2, RS232, RS485
- Supports coin in / out, bill acceptor, printer
- Supports CF card and other storage
- Supports VGA and LCD (high-resolution) and video-games (low-resolution)
  and TV (video)

The boot loader copies the game to RAM, arms the VR0 watchdog and spins
waiting for it to reset the CPU. The second pass (RAM contents preserved) then
starts the game.

Settings, bookkeeping and the clock are kept by an external device on the VR0
SIO port (probably the undumped QFP microcontroller near the DIP switches).
It's simulated here at a high level:
- host -> device: 0x80 starts a frame, then each byte is XORed with 0x95 and
  sent as two bytes holding the low nibble then the high nibble; 0x10 is sent
  while idle. A frame is <nibble count> <cmd> <addr lo> <addr hi> [data]
  <16-bit sum of the preceding bytes>.
- device -> host: bytes are XORed with 0x95. Writes are acknowledged with
  55 AA C5; reads return 55 AA <len + 2> <data> <16-bit sum of len and data>.
- commands: bit 4 set = read (single length byte), clear = write;
  bits 7-5 select one of several memory areas; 0xaf reads the BCD clock.

unkhaiwei is a player terminal of a networked Baccarat system: the DIP
switches set its terminal ID (1-15), and after boot the main loop only
processes messages from a host on the Ethernet port (raw frames, EtherType
0x6806 as read by the game). The host broadcasts round state (message 0x82,
0x50 byte block), polls terminals (0x8a, answered with 0x8b and 26 bytes of
terminal data) and adjusts terminal accounts (0xcc); 0x55, 0x7f, 0x8e, 0x8f
and 0xee are also handled. Without a host the game sits at the empty table
and ignores the keys. The host isn't dumped (it might be a different board
or program).

Inputs: the game scans 0x01500002, 0x01500000 and 0x0150000a (active low)
into key codes 0x00-0x14 through a table; keys named by the game include
Player, Banker, Tie, rules and the score up/down (上分/退分) buttons, but the
mapping can't be checked without a host. 0x0150000a bits 0 and 1 are also
counted separately with debounce, probably coin in and key in.

TODO:
- the watchdog period is a guess;
- the protocol of the SIO device beyond the commands used during boot and
  bookkeeping is unknown (0x6f sends 64 random-looking bytes, maybe a
  protection challenge; it's acknowledged and discarded);
- inputs only partially understood;
- hqdf not tested with the above (no ROMs on hand to confirm it uses the same
  hardware), it may need the second ROM area at 0x05000000 too;

*/

#include "emu.h"

#include "cpu/se3208/se3208.h"
#include "machine/dp8390.h"
#include "machine/nvram.h"
#include "machine/vrender0.h"
#include "sound/vrender0.h"
#include "video/vrender0.h"

#include "emupal.h"
#include "screen.h"
#include "speaker.h"


namespace {

class haiwei_state : public driver_device
{
public:
	haiwei_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_vr0soc(*this, "vr0soc"),
		m_nvram(*this, "nvram"),
		m_nic(*this, "nic")
	{}

	void hqdf(machine_config &config) ATTR_COLD;
	void baccarat(machine_config &config) ATTR_COLD;

	void init_haiwei() ATTR_COLD;

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	// memory areas of the SIO device: 8 selected by command bits 7-5, times 2 for low nibble 0xf / 0xa
	static constexpr unsigned SIO_AREAS = 16;
	static constexpr unsigned SIO_AREA_SIZE = 0x1000;

	required_device<se3208_device> m_maincpu;
	required_device<vrender0soc_device> m_vr0soc;
	required_device<nvram_device> m_nvram;
	required_device<rtl8019a_device> m_nic;

	emu_timer *m_watchdog_timer = nullptr;
	emu_timer *m_sio_rx_timer = nullptr;

	u32 m_pio = 0;
	u32 m_watchdog_ctrl = 0;
	u32 m_watchdog_count = 0;

	std::unique_ptr<u8 []> m_nic_ram;
	std::unique_ptr<u8 []> m_sio_ram;
	u8 m_sio_frame[0x100] = { };
	u16 m_sio_nibbles = 0;
	bool m_sio_in_frame = false;
	u8 m_sio_rx_buf[0x100] = { };
	u16 m_sio_rx_len = 0;
	u16 m_sio_rx_pos = 0;
	u8 m_sio_rx_data = 0;
	bool m_sio_rx_ready = false;

	void common_map(address_map &map) ATTR_COLD;
	void program_map(address_map &map) ATTR_COLD;
	void baccarat_map(address_map &map) ATTR_COLD;

	void decrypt(const char *region) ATTR_COLD;

	u32 pioldat_r();
	void pioldat_w(offs_t offset, u32 data, u32 mem_mask = ~0);
	u32 pioedat_r();

	u8 nic_r(offs_t offset);
	void nic_w(offs_t offset, u8 data);

	void watchdog_w(offs_t offset, u32 data, u32 mem_mask = ~0);
	void watchdog_restart();
	TIMER_CALLBACK_MEMBER(watchdog_expired);

	u32 sio_data_r(offs_t offset, u32 mem_mask = ~0);
	void sio_data_w(offs_t offset, u32 data, u32 mem_mask = ~0);
	u32 sio_status_r();
	TIMER_CALLBACK_MEMBER(sio_rx_next);
	void sio_process_frame();
	void sio_reply(const u8 *data, unsigned length);
	u8 &sio_mem(u8 cmd, u16 addr);
};


void haiwei_state::machine_start()
{
	m_watchdog_timer = timer_alloc(FUNC(haiwei_state::watchdog_expired), this);
	m_sio_rx_timer = timer_alloc(FUNC(haiwei_state::sio_rx_next), this);

	m_nic_ram = make_unique_clear<u8 []>(0x4000);
	m_sio_ram = std::make_unique<u8 []>(SIO_AREAS * SIO_AREA_SIZE);
	m_nvram->set_base(m_sio_ram.get(), SIO_AREAS * SIO_AREA_SIZE);

	save_item(NAME(m_pio));
	save_item(NAME(m_watchdog_ctrl));
	save_item(NAME(m_watchdog_count));
	save_pointer(NAME(m_nic_ram), 0x4000);
	save_pointer(NAME(m_sio_ram), SIO_AREAS * SIO_AREA_SIZE);
	save_item(NAME(m_sio_frame));
	save_item(NAME(m_sio_nibbles));
	save_item(NAME(m_sio_in_frame));
	save_item(NAME(m_sio_rx_buf));
	save_item(NAME(m_sio_rx_len));
	save_item(NAME(m_sio_rx_pos));
	save_item(NAME(m_sio_rx_data));
	save_item(NAME(m_sio_rx_ready));
}

void haiwei_state::machine_reset()
{
	// a watchdog reset only resets the CPU side, main RAM is kept (the boot loader relies on it)
	m_watchdog_ctrl = 0;
	m_watchdog_count = 0;
	m_watchdog_timer->adjust(attotime::never);

	m_sio_nibbles = 0;
	m_sio_in_frame = false;
	m_sio_rx_len = 0;
	m_sio_rx_pos = 0;
	m_sio_rx_ready = false;
	m_sio_rx_timer->adjust(attotime::never);
}

u32 haiwei_state::pioldat_r()
{
	return m_pio;
}

// PIO Latched output DATa Register
// TODO: bits 4 and 29 sets as output (latter flips between 1 and 0)
void haiwei_state::pioldat_w(offs_t offset, u32 data, u32 mem_mask)
{
	COMBINE_DATA(&m_pio);
}

// PIO External DATa Register
u32 haiwei_state::pioedat_r()
{
	return 0;
}


// RTL8019AS: one register per 0x100 bytes, NE2000-style data port after the registers
// TODO: data/reset port locations assumed from the NE2000 layout (no packets get that far without a host); interrupt not hooked up (polled)
u8 haiwei_state::nic_r(offs_t offset)
{
	unsigned const reg = offset >> 6;
	if (reg < 0x10)
		return m_nic->cs_read(reg);
	else if (reg == 0x10)
		return m_nic->remote_read();
	else if (reg == 0x1f && !machine().side_effects_disabled())
		m_nic->dp8390_reset(CLEAR_LINE);
	return 0;
}

void haiwei_state::nic_w(offs_t offset, u8 data)
{
	unsigned const reg = offset >> 6;
	if (reg < 0x10)
		m_nic->cs_write(reg, data);
	else if (reg == 0x10)
		m_nic->remote_write(data);
	else if (reg == 0x1f)
		m_nic->dp8390_reset(CLEAR_LINE);
}


// VR0 watchdog: 0x01800010 control (bit 0 enable), 0x01800014 count
// TODO: period unknown; this probably belongs in the VRENDER0_SOC device
void haiwei_state::watchdog_w(offs_t offset, u32 data, u32 mem_mask)
{
	if (offset == 0)
		COMBINE_DATA(&m_watchdog_ctrl);
	else
		COMBINE_DATA(&m_watchdog_count);

	watchdog_restart();
}

void haiwei_state::watchdog_restart()
{
	if (BIT(m_watchdog_ctrl, 0))
		m_watchdog_timer->adjust(attotime::from_ticks(u64(m_watchdog_count & 0xffff) + 1, m_maincpu->clock() / 1024));
	else
		m_watchdog_timer->adjust(attotime::never);
}

TIMER_CALLBACK_MEMBER(haiwei_state::watchdog_expired)
{
	logerror("watchdog reset\n");
	machine().schedule_soft_reset();
}


// SIO data register: reads return the received byte, writes send a byte to the external device
u32 haiwei_state::sio_data_r(offs_t offset, u32 mem_mask)
{
	if (!machine().side_effects_disabled())
		m_sio_rx_ready = false;

	return m_sio_rx_data;
}

void haiwei_state::sio_data_w(offs_t offset, u32 data, u32 mem_mask)
{
	if (!ACCESSING_BITS_0_7)
		return;

	u8 const byte = data & 0xff;

	if (byte == 0x80)
	{
		m_sio_in_frame = true;
		m_sio_nibbles = 0;
		return;
	}

	if (byte == 0x10 || !m_sio_in_frame) // idle
		return;

	if (byte & 0xf0)
	{
		logerror("SIO: unexpected byte %02x in frame\n", byte);
		m_sio_in_frame = false;
		return;
	}

	unsigned const index = m_sio_nibbles >> 1;
	if (m_sio_nibbles & 1)
		m_sio_frame[index] = (m_sio_frame[index] | (byte << 4)) ^ 0x95;
	else
		m_sio_frame[index] = byte;
	m_sio_nibbles++;

	// the first byte holds the frame length in nibbles
	if ((m_sio_nibbles >= 2) && (m_sio_nibbles >= m_sio_frame[0]))
	{
		m_sio_in_frame = false;
		sio_process_frame();
	}
	else if (m_sio_nibbles >= std::size(m_sio_frame) * 2)
	{
		logerror("SIO: frame overflow\n");
		m_sio_in_frame = false;
	}
}

// bit 0: receive data ready, bit 2: transmitter empty
u32 haiwei_state::sio_status_r()
{
	return 0x4 | (m_sio_rx_ready ? 0x1 : 0x0);
}

u8 &haiwei_state::sio_mem(u8 cmd, u16 addr)
{
	unsigned const area = ((cmd >> 5) & 7) | (((cmd & 0x0f) == 0x0a) ? 8 : 0);
	return m_sio_ram[area * SIO_AREA_SIZE + (addr & (SIO_AREA_SIZE - 1))];
}

void haiwei_state::sio_process_frame()
{
	unsigned const length = m_sio_frame[0] >> 1;
	if (length < 6)
	{
		logerror("SIO: short frame (%u bytes)\n", length);
		return;
	}

	u16 sum = 0;
	for (unsigned i = 0; i < length - 2; i++)
		sum += m_sio_frame[i];
	u16 const expected = m_sio_frame[length - 2] | (m_sio_frame[length - 1] << 8);
	if (sum != expected)
	{
		logerror("SIO: bad checksum %04x (expected %04x)\n", sum, expected);
		return;
	}

	u8 const cmd = m_sio_frame[1];
	u16 const addr = m_sio_frame[2] | (m_sio_frame[3] << 8);
	u8 const *const payload = &m_sio_frame[4];
	unsigned const payload_length = length - 6;

	if (cmd == 0xaf)
	{
		// clock: seconds, minutes, hours, day of week, day, month, year in BCD
		system_time systime;
		machine().current_datetime(systime);
		auto const bcd = [] (unsigned v) -> u8 { return ((v / 10) << 4) | (v % 10); };
		u8 const data[7] = {
				bcd(systime.local_time.second),
				bcd(systime.local_time.minute),
				bcd(systime.local_time.hour),
				bcd(systime.local_time.weekday),
				bcd(systime.local_time.mday),
				bcd(systime.local_time.month + 1),
				bcd(systime.local_time.year % 100) };
		sio_reply(data, std::size(data));
	}
	else if (BIT(cmd, 4))
	{
		// read
		unsigned const count = std::min<unsigned>(payload_length ? payload[0] : 0, std::size(m_sio_rx_buf) - 5);
		u8 data[0x100];
		for (unsigned i = 0; i < count; i++)
			data[i] = sio_mem(cmd, addr + i);
		sio_reply(data, count);
	}
	else
	{
		// write
		if (cmd == 0x6f)
			logerror("SIO: command 6f with %u bytes ignored\n", payload_length);
		else if ((cmd & 0x0f) != 0x0f && (cmd & 0x0f) != 0x0a)
			logerror("SIO: unknown command %02x addr %04x, %u bytes\n", cmd, addr, payload_length);
		else
			for (unsigned i = 0; i < payload_length; i++)
				sio_mem(cmd, addr + i) = payload[i];
		sio_reply(nullptr, 0);
	}
}

void haiwei_state::sio_reply(const u8 *data, unsigned length)
{
	m_sio_rx_len = 0;
	m_sio_rx_pos = 0;
	m_sio_rx_buf[m_sio_rx_len++] = 0x55;
	m_sio_rx_buf[m_sio_rx_len++] = 0xaa;

	if (!data)
	{
		m_sio_rx_buf[m_sio_rx_len++] = 0xc5; // ACK
	}
	else
	{
		u16 sum = length + 2;
		m_sio_rx_buf[m_sio_rx_len++] = length + 2;
		for (unsigned i = 0; i < length; i++)
		{
			m_sio_rx_buf[m_sio_rx_len++] = data[i];
			sum += data[i];
		}
		m_sio_rx_buf[m_sio_rx_len++] = sum & 0xff;
		m_sio_rx_buf[m_sio_rx_len++] = sum >> 8;
	}

	for (unsigned i = 0; i < m_sio_rx_len; i++)
		m_sio_rx_buf[i] ^= 0x95;

	// TODO: real baud rate unknown, roughly 9600 baud
	m_sio_rx_timer->adjust(attotime::from_usec(1000));
}

TIMER_CALLBACK_MEMBER(haiwei_state::sio_rx_next)
{
	if (m_sio_rx_pos >= m_sio_rx_len)
		return;

	m_sio_rx_data = m_sio_rx_buf[m_sio_rx_pos++];
	m_sio_rx_ready = true;
	m_vr0soc->int_req(3);

	m_sio_rx_timer->adjust(attotime::from_usec(1000));
}


void haiwei_state::common_map(address_map &map)
{
	map(0x01500000, 0x01500003).portr("IN0");
	map(0x01500004, 0x01500007).portr("DSW2");
	map(0x01500008, 0x0150000b).portr("IN1");
	map(0x01608000, 0x01609fff).rw(FUNC(haiwei_state::nic_r), FUNC(haiwei_state::nic_w)).umask32(0x000000ff);

	map(0x01800000, 0x01ffffff).m(m_vr0soc, FUNC(vrender0soc_device::regs_map));
	map(0x01800010, 0x01800017).w(FUNC(haiwei_state::watchdog_w));
	map(0x01802004, 0x01802007).rw(FUNC(haiwei_state::pioldat_r), FUNC(haiwei_state::pioldat_w));
	map(0x01802008, 0x0180200b).r(FUNC(haiwei_state::pioedat_r));
	map(0x01802804, 0x01802807).rw(FUNC(haiwei_state::sio_data_r), FUNC(haiwei_state::sio_data_w));
	map(0x01802810, 0x01802813).r(FUNC(haiwei_state::sio_status_r));

	map(0x02000000, 0x027fffff).ram().share("workram");

	map(0x03000000, 0x04ffffff).m(m_vr0soc, FUNC(vrender0soc_device::audiovideo_map));
}

void haiwei_state::program_map(address_map &map)
{
	common_map(map);

	map(0x00000000, 0x00bfffff).rom();
}

void haiwei_state::baccarat_map(address_map &map)
{
	common_map(map);

	map(0x00000000, 0x007fffff).rom();
	// game data archive in the third ROM, found by the game at this address
	map(0x05000000, 0x053fffff).rom().region("extrom", 0);
}


static INPUT_PORTS_START( hqdf )
	PORT_START("IN0")
	PORT_DIPNAME( 0x00000001, 0x00000001, "IN0" )
	PORT_DIPSETTING(          0x00000001, DEF_STR( Off ) )
	PORT_DIPSETTING(          0x00000000, DEF_STR( On ) )
	PORT_DIPNAME( 0x00000002, 0x00000002, DEF_STR( Unknown ) )
	PORT_DIPSETTING(          0x00000002, DEF_STR( Off ) )
	PORT_DIPSETTING(          0x00000000, DEF_STR( On ) )
	PORT_DIPNAME( 0x00000004, 0x00000004, DEF_STR( Unknown ) )
	PORT_DIPSETTING(          0x00000004, DEF_STR( Off ) )
	PORT_DIPSETTING(          0x00000000, DEF_STR( On ) )
	PORT_DIPNAME( 0x00000008, 0x00000008, DEF_STR( Unknown ) )
	PORT_DIPSETTING(          0x00000008, DEF_STR( Off ) )
	PORT_DIPSETTING(          0x00000000, DEF_STR( On ) )
	PORT_DIPNAME( 0x00000010, 0x00000010, DEF_STR( Unknown ) )
	PORT_DIPSETTING(          0x00000010, DEF_STR( Off ) )
	PORT_DIPSETTING(          0x00000000, DEF_STR( On ) )
	PORT_DIPNAME( 0x00000020, 0x00000020, DEF_STR( Unknown ) )
	PORT_DIPSETTING(          0x00000020, DEF_STR( Off ) )
	PORT_DIPSETTING(          0x00000000, DEF_STR( On ) )
	PORT_DIPNAME( 0x00000040, 0x00000040, DEF_STR( Unknown ) )
	PORT_DIPSETTING(          0x00000040, DEF_STR( Off ) )
	PORT_DIPSETTING(          0x00000000, DEF_STR( On ) )
	PORT_DIPNAME( 0x00000080, 0x00000080, DEF_STR( Unknown ) )
	PORT_DIPSETTING(          0x00000080, DEF_STR( Off ) )
	PORT_DIPSETTING(          0x00000000, DEF_STR( On ) )
	PORT_DIPNAME( 0x00010000, 0x00010000, "IN0-1" )
	PORT_DIPSETTING(          0x00010000, DEF_STR( Off ) )
	PORT_DIPSETTING(          0x00000000, DEF_STR( On ) )
	PORT_DIPNAME( 0x00020000, 0x00020000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(          0x00020000, DEF_STR( Off ) )
	PORT_DIPSETTING(          0x00000000, DEF_STR( On ) )
	PORT_DIPNAME( 0x00040000, 0x00040000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(          0x00040000, DEF_STR( Off ) )
	PORT_DIPSETTING(          0x00000000, DEF_STR( On ) )
	PORT_DIPNAME( 0x00080000, 0x00080000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(          0x00080000, DEF_STR( Off ) )
	PORT_DIPSETTING(          0x00000000, DEF_STR( On ) )
	PORT_DIPNAME( 0x00100000, 0x00100000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(          0x00100000, DEF_STR( Off ) )
	PORT_DIPSETTING(          0x00000000, DEF_STR( On ) )
	PORT_DIPNAME( 0x00200000, 0x00200000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(          0x00200000, DEF_STR( Off ) )
	PORT_DIPSETTING(          0x00000000, DEF_STR( On ) )
	PORT_DIPNAME( 0x00400000, 0x00400000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(          0x00400000, DEF_STR( Off ) )
	PORT_DIPSETTING(          0x00000000, DEF_STR( On ) )
	PORT_DIPNAME( 0x00800000, 0x00800000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(          0x00800000, DEF_STR( Off ) )
	PORT_DIPSETTING(          0x00000000, DEF_STR( On ) )
	PORT_BIT( 0xff00ff00, IP_ACTIVE_LOW, IPT_UNKNOWN )

	PORT_START("IN1")
	PORT_DIPNAME( 0x00010000, 0x00010000, "IN1" )
	PORT_DIPSETTING(          0x00010000, DEF_STR( Off ) )
	PORT_DIPSETTING(          0x00000000, DEF_STR( On ) )
	PORT_DIPNAME( 0x00020000, 0x00020000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(          0x00020000, DEF_STR( Off ) )
	PORT_DIPSETTING(          0x00000000, DEF_STR( On ) )
	PORT_DIPNAME( 0x00040000, 0x00040000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(          0x00040000, DEF_STR( Off ) )
	PORT_DIPSETTING(          0x00000000, DEF_STR( On ) )
	PORT_DIPNAME( 0x00080000, 0x00080000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(          0x00080000, DEF_STR( Off ) )
	PORT_DIPSETTING(          0x00000000, DEF_STR( On ) )
	PORT_DIPNAME( 0x00100000, 0x00100000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(          0x00100000, DEF_STR( Off ) )
	PORT_DIPSETTING(          0x00000000, DEF_STR( On ) )
	PORT_DIPNAME( 0x00200000, 0x00200000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(          0x00200000, DEF_STR( Off ) )
	PORT_DIPSETTING(          0x00000000, DEF_STR( On ) )
	PORT_DIPNAME( 0x00400000, 0x00400000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(          0x00400000, DEF_STR( Off ) )
	PORT_DIPSETTING(          0x00000000, DEF_STR( On ) )
	PORT_DIPNAME( 0x00800000, 0x00800000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(          0x00800000, DEF_STR( Off ) )
	PORT_DIPSETTING(          0x00000000, DEF_STR( On ) )
	PORT_BIT( 0xff00ff00, IP_ACTIVE_LOW, IPT_UNKNOWN )

	// read bit-reversed: switch 1 is bit 7
	PORT_DIPNAME( 0x000000f0, 0x00000070, "Terminal ID" ) PORT_DIPLOCATION("SW1:1,2,3,4") // unkhaiwei stops with "games num error" if 0
	PORT_DIPSETTING(          0x000000f0, "0 (invalid)" )
	PORT_DIPSETTING(          0x00000070, "1" )
	PORT_DIPSETTING(          0x000000b0, "2" )
	PORT_DIPSETTING(          0x00000030, "3" )
	PORT_DIPSETTING(          0x000000d0, "4" )
	PORT_DIPSETTING(          0x00000050, "5" )
	PORT_DIPSETTING(          0x00000090, "6" )
	PORT_DIPSETTING(          0x00000010, "7" )
	PORT_DIPSETTING(          0x000000e0, "8" )
	PORT_DIPSETTING(          0x00000060, "9" )
	PORT_DIPSETTING(          0x000000a0, "10" )
	PORT_DIPSETTING(          0x00000020, "11" )
	PORT_DIPSETTING(          0x000000c0, "12" )
	PORT_DIPSETTING(          0x00000040, "13" )
	PORT_DIPSETTING(          0x00000080, "14" )
	PORT_DIPSETTING(          0x00000000, "15" )
	PORT_DIPUNKNOWN_DIPLOC( 0x00000008, 0x00000008, "SW1:5" ) // unkhaiwei: when on, calls extra init code and stays at the logo
	PORT_DIPNAME( 0x00000004, 0x00000004, "Pair bets" ) PORT_DIPLOCATION("SW1:6") // unkhaiwei: adds Player pair / Banker pair spots to the table
	PORT_DIPSETTING(          0x00000004, DEF_STR( Off ) )
	PORT_DIPSETTING(          0x00000000, DEF_STR( On ) )
	PORT_DIPUNKNOWN_DIPLOC( 0x00000002, 0x00000002, "SW1:7" )
	PORT_DIPUNKNOWN_DIPLOC( 0x00000001, 0x00000001, "SW1:8" )

	// read bit-reversed like the DIP switches, by the same kind of routine
	PORT_START("DSW2")
	PORT_BIT( 0xffffffff, IP_ACTIVE_LOW, IPT_UNKNOWN )
INPUT_PORTS_END


void haiwei_state::hqdf(machine_config &config)
{
	SE3208(config, m_maincpu, 14.318181_MHz_XTAL * 3); // multiplier not verified
	m_maincpu->set_addrmap(AS_PROGRAM, &haiwei_state::program_map);
	m_maincpu->iackx_cb().set(m_vr0soc, FUNC(vrender0soc_device::irq_callback));

	VRENDER0_SOC(config, m_vr0soc, 14.318181_MHz_XTAL * 6); // multiplier not verified
	m_vr0soc->set_host_space_tag(m_maincpu, AS_PROGRAM);
	m_vr0soc->int_callback().set_inputline(m_maincpu, se3208_device::SE3208_INT);
	m_vr0soc->set_external_vclk(28.636363_MHz_XTAL);

	NVRAM(config, m_nvram, nvram_device::DEFAULT_ALL_0); // in the SIO device

	RTL8019A(config, m_nic, 0);
	m_nic->mem_read_callback().set([this] (offs_t offset) -> u8 { return ((offset & 0xc000) == 0x4000) ? m_nic_ram[offset & 0x3fff] : 0xff; });
	m_nic->mem_write_callback().set([this] (offs_t offset, u8 data) { if ((offset & 0xc000) == 0x4000) m_nic_ram[offset & 0x3fff] = data; });

	SPEAKER(config, "speaker", 2).front();
	m_vr0soc->add_route(0, "speaker", 1.0, 0);
	m_vr0soc->add_route(1, "speaker", 1.0, 1);
}

void haiwei_state::baccarat(machine_config &config)
{
	hqdf(config);

	m_maincpu->set_addrmap(AS_PROGRAM, &haiwei_state::baccarat_map);
}


// 环球大富翁 (Huánqiú Dàfùwēng)
ROM_START( hqdf )
	ROM_REGION32_LE( 0xc00000, "maincpu", ROMREGION_ERASE00 ) // all ST27C160
	ROM_LOAD( "1", 0x000000, 0x200000, CRC(4ce1c5eb) SHA1(df69037a44ce9b8944ddb38d7c226d1acf57eaca) )
	ROM_LOAD( "2", 0x200000, 0x200000, CRC(9f5fbd56) SHA1(518937dafe754a575987913fa19c5c135930cecc) )
	ROM_LOAD( "3", 0x400000, 0x200000, CRC(4edea690) SHA1(ba919577418e9f245113160adbdca9d63c54822e) )
ROM_END

ROM_START( unkhaiwei ) // title screen says BACCARAT
	ROM_REGION32_LE( 0x800000, "maincpu", 0 )
	ROM_LOAD( "1", 0x000000, 0x400000, CRC(7850ff4d) SHA1(f247965d0b79ab3cb85efef8a0ac4d788f63ff7b) )
	ROM_LOAD( "2", 0x400000, 0x400000, CRC(d123e649) SHA1(3c6aadb61acfa7c4536dc37b1e1505bcac4db504) )

	ROM_REGION32_LE( 0x400000, "extrom", 0 )
	ROM_LOAD( "3", 0x000000, 0x400000, CRC(6b4cad4e) SHA1(6f3fca502e954d43a9fa90fda63c2346a8f39515) ) // only the first 512KiB is used
ROM_END


void haiwei_state::init_haiwei()
{
	decrypt("maincpu");
	if (memregion("extrom"))
		decrypt("extrom");
}

void haiwei_state::decrypt(const char *region)
{
	u8 *rom = memregion(region)->base();
	u32 length = memregion(region)->bytes();

	std::vector<u8> buffer(length);

	memcpy(&buffer[0], rom, length);

	for (int i = 0; i < length; i++)
		rom[i] = buffer[bitswap<24>(i, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 7, 8, 6, 5, 4, 3, 2, 1, 0)];

	for (int i = 0; i < length; i++)
	{
		if (i & 0x01)
			rom[i] = bitswap<8>(rom[i], 6, 7, 5, 4, 0, 1, 2, 3);
		else
			rom[i] = bitswap<8>(rom[i], 6, 7, 5, 4, 3, 2, 1, 0);
	}
}

} // anonymous namespace


GAME( 2006, hqdf,      0, hqdf,     hqdf, haiwei_state, init_haiwei, ROT0, "Hai Wei Technology", "Huanqiu Dafuweng",                MACHINE_NO_SOUND | MACHINE_NOT_WORKING )
GAME( 200?, unkhaiwei, 0, baccarat, hqdf, haiwei_state, init_haiwei, ROT0, "Hai Wei Technology", "Baccarat (Hai Wei Technology, terminal)", MACHINE_NO_SOUND | MACHINE_NOT_WORKING ) // needs the network host to run rounds
