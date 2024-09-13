// license:BSD-3-Clause
// copyright-holders:AJR
/***********************************************************************************************************************************

    TeleVideo 950 101-key serial keyboard emulation.

************************************************************************************************************************************/

#include "emu.h"
#include "tv950kb.h"

DEFINE_DEVICE_TYPE(TV950_KEYBOARD, tv950kb_device, "tv950kb", "TeleVideo 950 Keyboard")

tv950kb_device::tv950kb_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, TV950_KEYBOARD, tag, owner, clock)
	, m_tx_cb(*this)
	, m_mcu(*this, "mcu")
	, m_beeper(*this, "beeper")
	, m_keys(*this, "ROW%u", 0U)
{
}

void tv950kb_device::device_start()
{
}

void tv950kb_device::rx_w(int state)
{
	m_beeper->level_w(state);
}

u8 tv950kb_device::keys_r()
{
	const u16 row_select = m_mcu->p2_r() | m_mcu->p1_r() << 8;
	u8 result = 0xff;

	for (int n = 0; n < 13; n++)
		if (!BIT(row_select, n))
			result &= m_keys[n]->read();

	return result;
}

void tv950kb_device::tx_w(int state)
{
	m_tx_cb(state);
}

void tv950kb_device::rw_map(address_map &map)
{
	map(0x00, 0x00).mirror(0xff).r(FUNC(tv950kb_device::keys_r)).nopw();
}

void tv950kb_device::device_add_mconfig(machine_config &config)
{
	I8748(config, m_mcu, 5.7143_MHz_XTAL);
	m_mcu->p1_in_cb().set_ioport("FUNCT");
	m_mcu->p1_out_cb().set(FUNC(tv950kb_device::tx_w)).bit(7);
	m_mcu->t0_in_cb().set_ioport("CTRL").bit(0);
	m_mcu->t1_in_cb().set_ioport("CTRL").bit(1);
	m_mcu->set_addrmap(AS_IO, &tv950kb_device::rw_map);

	SPEAKER(config, "mono").front_center();
	SPEAKER_SOUND(config, m_beeper).add_route(ALL_OUTPUTS, "mono", 0.50);
}

static INPUT_PORTS_START(tv950kb)
	PORT_START("ROW0")
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('1') PORT_CHAR('!')       PORT_CODE(KEYCODE_1)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('2') PORT_CHAR('@')       PORT_CODE(KEYCODE_2)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('3') PORT_CHAR('#')       PORT_CODE(KEYCODE_3)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('4') PORT_CHAR('$')       PORT_CODE(KEYCODE_4)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('5') PORT_CHAR('%')       PORT_CODE(KEYCODE_5)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('6') PORT_CHAR('^')       PORT_CODE(KEYCODE_6)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('7') PORT_CHAR('&')       PORT_CODE(KEYCODE_7)
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('8') PORT_CHAR('*')       PORT_CODE(KEYCODE_8)

	PORT_START("ROW1")
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('9') PORT_CHAR('(')       PORT_CODE(KEYCODE_9)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('0') PORT_CHAR(')')       PORT_CODE(KEYCODE_0)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F7  Ins Ch")       PORT_CHAR(UCHAR_MAMEKEY(F7))        PORT_CODE(KEYCODE_F12) // "Char Insert" according to operator's manual
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F8  Del Ch")       PORT_CHAR(UCHAR_MAMEKEY(F8))        PORT_CODE(KEYCODE_F13) // "Char Delete" according to operator's manual
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Loc  Ins Ln")                                          PORT_CODE(KEYCODE_F14) // "Line Insert" according to operator's manual
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Msg  Del Ln")                                          PORT_CODE(KEYCODE_F15) // "Line Delete" according to operator's manual
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('-') PORT_CHAR('_')       PORT_CODE(KEYCODE_MINUS)
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('=') PORT_CHAR('+')       PORT_CODE(KEYCODE_EQUALS)

	PORT_START("ROW2")
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('`') PORT_CHAR('~')       PORT_CODE(KEYCODE_TILDE)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('\\') PORT_CHAR('|')      PORT_CODE(KEYCODE_BACKSLASH)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Back Space")       PORT_CHAR(0x08)                     PORT_CODE(KEYCODE_BACKSPACE)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR(0x09)                     PORT_CODE(KEYCODE_TAB)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('q') PORT_CHAR('Q')       PORT_CODE(KEYCODE_Q)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('w') PORT_CHAR('W')       PORT_CODE(KEYCODE_W)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('e') PORT_CHAR('E')       PORT_CODE(KEYCODE_E)
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('r') PORT_CHAR('R')       PORT_CODE(KEYCODE_R)

	PORT_START("ROW3")
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('t') PORT_CHAR('T')       PORT_CODE(KEYCODE_T)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('y') PORT_CHAR('Y')       PORT_CODE(KEYCODE_Y)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('u') PORT_CHAR('U')       PORT_CODE(KEYCODE_U)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('i') PORT_CHAR('I')       PORT_CODE(KEYCODE_I)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('o') PORT_CHAR('O')       PORT_CODE(KEYCODE_O)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('p') PORT_CHAR('P')       PORT_CODE(KEYCODE_P)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('[') PORT_CHAR(']')       PORT_CODE(KEYCODE_OPENBRACE)
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Line Feed")        PORT_CHAR(0x0a)

	PORT_START("ROW4")
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('a') PORT_CHAR('A')       PORT_CODE(KEYCODE_A)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('s') PORT_CHAR('S')       PORT_CODE(KEYCODE_S)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('d') PORT_CHAR('D')       PORT_CODE(KEYCODE_D)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('f') PORT_CHAR('F')       PORT_CODE(KEYCODE_F)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('g') PORT_CHAR('G')       PORT_CODE(KEYCODE_G)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('h') PORT_CHAR('H')       PORT_CODE(KEYCODE_H)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('j') PORT_CHAR('J')       PORT_CODE(KEYCODE_J)
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('k') PORT_CHAR('K')       PORT_CODE(KEYCODE_K)

	PORT_START("ROW5")
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('l') PORT_CHAR('L')       PORT_CODE(KEYCODE_L)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR(';') PORT_CHAR(':')       PORT_CODE(KEYCODE_COLON)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('\'') PORT_CHAR('"')      PORT_CODE(KEYCODE_QUOTE)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Back Tab") // below Ctrl
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('z') PORT_CHAR('Z')       PORT_CODE(KEYCODE_Z)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('x') PORT_CHAR('X')       PORT_CODE(KEYCODE_X)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('c') PORT_CHAR('C')       PORT_CODE(KEYCODE_C)
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('v') PORT_CHAR('V')       PORT_CODE(KEYCODE_V)

	PORT_START("ROW6")
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('b') PORT_CHAR('B')       PORT_CODE(KEYCODE_B)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('n') PORT_CHAR('N')       PORT_CODE(KEYCODE_N)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('m') PORT_CHAR('M')       PORT_CODE(KEYCODE_M)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR(',') PORT_CHAR('<')       PORT_CODE(KEYCODE_COMMA)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('.') PORT_CHAR('>')       PORT_CODE(KEYCODE_STOP)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('/') PORT_CHAR('?')       PORT_CODE(KEYCODE_SLASH)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR('{') PORT_CHAR('}')       PORT_CODE(KEYCODE_CLOSEBRACE) // to right of Shift
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR(UCHAR_MAMEKEY(DEL))       PORT_CODE(KEYCODE_DEL)

	PORT_START("ROW7")
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR(' ')                      PORT_CODE(KEYCODE_SPACE)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR(UCHAR_MAMEKEY(DOWN))      PORT_CODE(KEYCODE_DOWN)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR(UCHAR_MAMEKEY(UP))        PORT_CODE(KEYCODE_UP)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR(UCHAR_MAMEKEY(LEFT))      PORT_CODE(KEYCODE_LEFT)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR(UCHAR_MAMEKEY(RIGHT))     PORT_CODE(KEYCODE_RIGHT)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR(UCHAR_MAMEKEY(7_PAD))     PORT_CODE(KEYCODE_7_PAD)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR(UCHAR_MAMEKEY(8_PAD))     PORT_CODE(KEYCODE_8_PAD)
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR(UCHAR_MAMEKEY(9_PAD))     PORT_CODE(KEYCODE_9_PAD)

	PORT_START("ROW8")
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR(UCHAR_MAMEKEY(4_PAD))     PORT_CODE(KEYCODE_4_PAD)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR(UCHAR_MAMEKEY(5_PAD))     PORT_CODE(KEYCODE_5_PAD)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR(UCHAR_MAMEKEY(6_PAD))     PORT_CODE(KEYCODE_6_PAD)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR(UCHAR_MAMEKEY(1_PAD))     PORT_CODE(KEYCODE_1_PAD)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR(UCHAR_MAMEKEY(2_PAD))     PORT_CODE(KEYCODE_2_PAD)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR(UCHAR_MAMEKEY(3_PAD))     PORT_CODE(KEYCODE_3_PAD)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR(UCHAR_MAMEKEY(COMMA_PAD)) PORT_CODE(KEYCODE_COMMA_PAD)
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR(UCHAR_MAMEKEY(0_PAD))     PORT_CODE(KEYCODE_0_PAD)

	PORT_START("ROW9")
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR(UCHAR_MAMEKEY(DEL_PAD))   PORT_CODE(KEYCODE_DEL_PAD)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Keypad _") // on lowest row of keypad
	PORT_BIT(0x3f, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("ROW10")
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Esc  Loc Esc")     PORT_CHAR(0x1b)                     PORT_CODE(KEYCODE_ESC)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Cmd Line") // above KP 7; "Line Erase" according to operator's manual
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Start")    // above KP 8; "Page Erase" according to operator's manual
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Step")     // above KP 9; "Send" according to operator's manual
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Home")             PORT_CHAR(UCHAR_MAMEKEY(HOME))      PORT_CODE(KEYCODE_RALT)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Return")           PORT_CHAR(0x0d)                     PORT_CODE(KEYCODE_ENTER)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Field Select") // below Back Space; "Clear Space" according to operator's manual
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR(UCHAR_MAMEKEY(ENTER_PAD)) PORT_CODE(KEYCODE_ENTER_PAD) // keypad lower left corner

	PORT_START("ROW11")
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME(u8"Help  \u2190Wd")                                     PORT_CODE(KEYCODE_F1)  // ←Wd "F1" according to operator's manual
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME(u8"Top Menu  \u2192Wd")                                 PORT_CODE(KEYCODE_F2)  // →Wd "F2" according to operator's manual
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME(u8"Move  \u2191Scroll")                                 PORT_CODE(KEYCODE_F3)  // ↑Scroll "F3" according to operator's manual
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME(u8"  \u2193Scroll")                                     PORT_CODE(KEYCODE_F4)  // ↓Scroll "F4" according to operator's manual
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME(u8"F1  \u2191Scrn") PORT_CHAR(UCHAR_MAMEKEY(F1))        PORT_CODE(KEYCODE_F5)  // ↑Scrn "F5" according to operator's manual
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME(u8"F2  \u2193Scrn") PORT_CHAR(UCHAR_MAMEKEY(F2))        PORT_CODE(KEYCODE_F6)  // ↓Scrn "F6" according to operator's manual
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F3  Del Wo")       PORT_CHAR(UCHAR_MAMEKEY(F3))        PORT_CODE(KEYCODE_F7)  // "F7" according to operator's manual
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD)                               PORT_CHAR(UCHAR_MAMEKEY(F4))        PORT_CODE(KEYCODE_F8)  // "F8" according to operator's manual

	PORT_START("ROW12")
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Blank key")                                            PORT_CODE(KEYCODE_F9)  // "F9" according to operator's manual
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F5  EOF")          PORT_CHAR(UCHAR_MAMEKEY(F5))        PORT_CODE(KEYCODE_F10) // "F10" according to operator's manual
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F6  BOF")          PORT_CHAR(UCHAR_MAMEKEY(F6))        PORT_CODE(KEYCODE_F11) // "F11" according to operator's manual
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("No Scroll  Set Up") // top left corner
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Print") // to left of Funct
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Break") // above Del
	PORT_BIT(0x03, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("CTRL")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Ctrl")             PORT_CHAR(UCHAR_SHIFT_2)            PORT_CODE(KEYCODE_LCONTROL)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Shift")            PORT_CHAR(UCHAR_SHIFT_1)            PORT_CODE(KEYCODE_LSHIFT) PORT_CODE(KEYCODE_RSHIFT)

	PORT_START("FUNCT")
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Funct")            PORT_CHAR(UCHAR_MAMEKEY(LALT))      PORT_CODE(KEYCODE_LALT)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Alpha Lock")       PORT_CHAR(UCHAR_MAMEKEY(CAPSLOCK))  PORT_CODE(KEYCODE_CAPSLOCK) PORT_TOGGLE
	PORT_BIT(0x9f, IP_ACTIVE_LOW, IPT_UNUSED)
INPUT_PORTS_END

ioport_constructor tv950kb_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(tv950kb);
}

ROM_START(tv950kb)
	ROM_REGION(0x400, "mcu", 0)
	ROM_LOAD("950kbd_8748_pn52080723-02.bin", 0x000, 0x400, CRC(11c8f22c) SHA1(99e73e9c74b10055733e89b92adbc5bf7f4ff338))
ROM_END

const tiny_rom_entry *tv950kb_device::device_rom_region() const
{
	return ROM_NAME(tv950kb);
}
