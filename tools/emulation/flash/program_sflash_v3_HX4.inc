//
// Copyright: (c) 2013 Broadcom Corp.
// All Rights Reserved.
//

wait=on
error=abort

//
// User configurations
//
ADD unsigned int FLASH_START_OFFSET             = 0x0
ADD unsigned int FORCE_2_STEP_FOR_BOOT          = 1

//
// System/chip configurations
//
ADD unsigned int USE_DDR                        = 0
ADD unsigned int RAM_BUFFER_START               = 0x1B000000
ADD unsigned int RAM_BUFFER_LENGTH              = 512*1024
ADD unsigned int FLASH_ADDRESS_BASE             = 0x1E000000
ADD unsigned int QSPI_REGISTER_BASE             = 0x18027000
ADD unsigned int DDR_CTL_REGISTER_BASE          = 0x18010000
ADD unsigned int DDR_PHY_CTL_REGISTER_BASE      = 0x18010800
ADD unsigned int DDR_PHY_WL0_REGISTER_BASE      = 0x18010A00
ADD unsigned int DDR_S1_IDM_REGISTER_BASE       = 0x18109000
ADD unsigned int DDR_S2_IDM_REGISTER_BASE       = 0x1810A000
ADD unsigned int DDR_VOL_SEL_REGISTER           = 0
ADD unsigned int DDR_VOL_SEL_DDR3               = 0
ADD unsigned int L2CC_REGISTER_BASE             = 0x19022000

//
// Global variables and debugging configurations
//
ADD unsigned int ERASE_BEFORE_PROGRAM           = 1
ADD unsigned int EXECUTION_RESERVED_SIZE        = 1024
ADD unsigned char JEDEC[5]

// Delete symbols to avoid complaint for duplicated definitions
define /R sflash_delete_symbols(dummy)
int dummy;
{
    if (isalive(FLASH_START_OFFSET) == 1) {
        $delete FLASH_START_OFFSET$;
    }
    if (isalive(FORCE_2_STEP_FOR_BOOT) == 1) {
        $delete FORCE_2_STEP_FOR_BOOT$;
    }
    if (isalive(USE_DDR) == 1) {
        $delete USE_DDR$;
    }
    if (isalive(RAM_BUFFER_START) == 1) {
        $delete RAM_BUFFER_START$;
    }
    if (isalive(RAM_BUFFER_LENGTH) == 1) {
        $delete RAM_BUFFER_LENGTH$;
    }
    if (isalive(FLASH_ADDRESS_BASE) == 1) {
        $delete FLASH_ADDRESS_BASE$;
    }
    if (isalive(QSPI_REGISTER_BASE) == 1) {
        $delete QSPI_REGISTER_BASE$;
    }
    if (isalive(DDR_CTL_REGISTER_BASE) == 1) {
        $delete DDR_CTL_REGISTER_BASE$;
    }
    if (isalive(DDR_PHY_CTL_REGISTER_BASE) == 1) {
        $delete DDR_PHY_CTL_REGISTER_BASE$;
    }
    if (isalive(DDR_PHY_WL0_REGISTER_BASE) == 1) {
        $delete DDR_PHY_WL0_REGISTER_BASE$;
    }
    if (isalive(DDR_S1_IDM_REGISTER_BASE) == 1) {
        $delete DDR_S1_IDM_REGISTER_BASE$;
    }
    if (isalive(DDR_S2_IDM_REGISTER_BASE) == 1) {
        $delete DDR_S2_IDM_REGISTER_BASE$;
    }
    if (isalive(DDR_VOL_SEL_REGISTER) == 1) {
        $delete DDR_VOL_SEL_REGISTER$;
    }
    if (isalive(DDR_VOL_SEL_DDR3) == 1) {
        $delete DDR_VOL_SEL_DDR3$;
    }
    if (isalive(L2CC_REGISTER_BASE) == 1) {
        $delete L2CC_REGISTER_BASE$;
    }
    if (isalive(ERASE_BEFORE_PROGRAM) == 1) {
        $delete ERASE_BEFORE_PROGRAM$;
    }
    if (isalive(EXECUTION_RESERVED_SIZE) == 1) {
        $delete EXECUTION_RESERVED_SIZE$;
    }
    if (isalive(JEDEC) == 1) {
        $delete JEDEC$;
    }
}
.

define /R ddr_ctrl_reg_write(entry, value)
unsigned int entry;
unsigned int value;
{
    *(unsigned int *)(DDR_CTL_REGISTER_BASE + entry * 4) = value;
}
.

//
// DDR initialization
// 
define /R ddr_init(dummy)
int dummy;
{
    unsigned int i;
    unsigned int ddr3;

    // For DDR3, configure voltage to 1.5v
    ddr3 = 0;
    if ((*(unsigned int *)(DDR_S1_IDM_REGISTER_BASE + 0x500)) & 1) {
        ddr3 = 1;
        if (DDR_VOL_SEL_REGISTER != 0) {
            *(unsigned int *)DDR_VOL_SEL_REGISTER |= DDR_VOL_SEL_DDR3;
        }
    }
    
    // Enable DDR core
    *(unsigned int *)(DDR_S1_IDM_REGISTER_BASE + 0x800) = 0;
    *(unsigned int *)(DDR_S2_IDM_REGISTER_BASE + 0x800) = 0;
    *(unsigned int *)(DDR_S1_IDM_REGISTER_BASE + 0x408) &= 0xf000ffff;
    *(unsigned int *)(DDR_S1_IDM_REGISTER_BASE + 0x408) |= 0x01900000;
    
    // Wait for DDR PHY up
    while (*(unsigned int *)DDR_PHY_CTL_REGISTER_BASE == 0);
    
    // Wait for DDR PHY PLL lock
    *(unsigned int *)(DDR_PHY_CTL_REGISTER_BASE + 0x14) = 0x00000010;
    for(i=0; i<0x1400; i++) {
        if (*(unsigned int *)(DDR_PHY_CTL_REGISTER_BASE + 0x10) & 1) {
            break;
        }
    }
    
    *(unsigned int *)(DDR_PHY_CTL_REGISTER_BASE + 0x160) = 0x00000002;
    *(unsigned int *)(DDR_PHY_CTL_REGISTER_BASE + 0x1ac) = ddr3 | 2;
    *(unsigned int *)(DDR_PHY_CTL_REGISTER_BASE + 0x3c) = 0x00100000;
    *(unsigned int *)(DDR_PHY_CTL_REGISTER_BASE + 0x48) = 0x00000101;
    
    // Wait for DDY PHY calibration lock
    for(i=0; i<0x1400; i++) {
        if (*(unsigned int *)(DDR_PHY_CTL_REGISTER_BASE + 0x4C) & 1) {
            break;
        }
    }
    
    // Override calibration if not locked
    if ((*(unsigned int *)(DDR_PHY_CTL_REGISTER_BASE + 0x4C) & 2) == 0) {
        *(unsigned int *)(DDR_PHY_CTL_REGISTER_BASE + 0x34) = 0x0001003f;
        *(unsigned int *)(DDR_PHY_WL0_REGISTER_BASE + 0x04) = 0x0003003f;
        *(unsigned int *)(DDR_PHY_WL0_REGISTER_BASE + 0x10) = 0x0003003f;
        *(unsigned int *)(DDR_PHY_WL0_REGISTER_BASE + 0x14) = 0x0003003f;
        *(unsigned int *)(DDR_PHY_WL0_REGISTER_BASE + 0x18) = 0x0003003f;
        *(unsigned int *)(DDR_PHY_WL0_REGISTER_BASE + 0x1c) = 0x0003003f;
        *(unsigned int *)(DDR_PHY_WL0_REGISTER_BASE + 0x20) = 0x0003003f;
        *(unsigned int *)(DDR_PHY_WL0_REGISTER_BASE + 0x24) = 0x0003003f;
        *(unsigned int *)(DDR_PHY_WL0_REGISTER_BASE + 0x28) = 0x0003003f;
        *(unsigned int *)(DDR_PHY_WL0_REGISTER_BASE + 0x2c) = 0x0003003f;
        *(unsigned int *)(DDR_PHY_WL0_REGISTER_BASE + 0x30) = 0x0003003f;
        *(unsigned int *)(DDR_PHY_WL0_REGISTER_BASE + 0xa4) = 0x0003003f;
        *(unsigned int *)(DDR_PHY_WL0_REGISTER_BASE + 0xb0) = 0x0003003f;
        *(unsigned int *)(DDR_PHY_WL0_REGISTER_BASE + 0xb4) = 0x0003003f;
        *(unsigned int *)(DDR_PHY_WL0_REGISTER_BASE + 0xb8) = 0x0003003f;
        *(unsigned int *)(DDR_PHY_WL0_REGISTER_BASE + 0xbc) = 0x0003003f;
        *(unsigned int *)(DDR_PHY_WL0_REGISTER_BASE + 0xc0) = 0x0003003f;
        *(unsigned int *)(DDR_PHY_WL0_REGISTER_BASE + 0xc4) = 0x0003003f;
        *(unsigned int *)(DDR_PHY_WL0_REGISTER_BASE + 0xc8) = 0x0003003f;
        *(unsigned int *)(DDR_PHY_WL0_REGISTER_BASE + 0xcc) = 0x0003003f;
        *(unsigned int *)(DDR_PHY_WL0_REGISTER_BASE + 0xd0) = 0x0003003f;
        *(unsigned int *)(DDR_PHY_WL0_REGISTER_BASE + 0x08) = 0x0003003f;
        *(unsigned int *)(DDR_PHY_WL0_REGISTER_BASE + 0x0c) = 0x0003003f;
        *(unsigned int *)(DDR_PHY_WL0_REGISTER_BASE + 0xa8) = 0x0003003f;
        *(unsigned int *)(DDR_PHY_WL0_REGISTER_BASE + 0xac) = 0x0003003f;
    }
    
    // Fill common registers (for both DDR2 and DDR3)
    ddr_ctrl_reg_write(    14,    0x05051001    );
    ddr_ctrl_reg_write(    15,    0x000a0a05    );
    ddr_ctrl_reg_write(    36,    0x01000000    );
    ddr_ctrl_reg_write(    37,    0x10000000    );
    ddr_ctrl_reg_write(    38,    0x00100400    );
    ddr_ctrl_reg_write(    39,    0x00000400    );
    ddr_ctrl_reg_write(    40,    0x00000100    );
    ddr_ctrl_reg_write(    42,    0x00000001    );
    ddr_ctrl_reg_write(    61,    0x00010100    );
    ddr_ctrl_reg_write(    78,    0x01000200    );
    ddr_ctrl_reg_write(    79,    0x02000040    );
    ddr_ctrl_reg_write(    80,    0x00400100    );
    ddr_ctrl_reg_write(    81,    0x00000200    );
    ddr_ctrl_reg_write(    83,    0x01ffff0a    );
    ddr_ctrl_reg_write(    84,    0x01010101    );
    ddr_ctrl_reg_write(    85,    0x03010101    );
    ddr_ctrl_reg_write(    86,    0x01000003    );
    ddr_ctrl_reg_write(    87,    0x0000010c    );
    ddr_ctrl_reg_write(    88,    0x00010000    );
    ddr_ctrl_reg_write(    112,    0x00000200   );
    ddr_ctrl_reg_write(    116,    0x19000000   );
    ddr_ctrl_reg_write(    117,    0x00000028   );
    ddr_ctrl_reg_write(    119,    0x00010001   );
    ddr_ctrl_reg_write(    120,    0x00010001   );
    ddr_ctrl_reg_write(    121,    0x00010001   );
    ddr_ctrl_reg_write(    122,    0x00010001   );
    ddr_ctrl_reg_write(    123,    0x00010001   );
    ddr_ctrl_reg_write(    130,    0x00000001   );
    ddr_ctrl_reg_write(    139,    0x00000001   );
    ddr_ctrl_reg_write(    148,    0xffff0001   );
    ddr_ctrl_reg_write(    149,    0x00ffff00   );
    ddr_ctrl_reg_write(    150,    0x0000ffff   );
    ddr_ctrl_reg_write(    152,    0x03030303   );
    ddr_ctrl_reg_write(    153,    0x03030303   );
    ddr_ctrl_reg_write(    156,    0x02006400   );
    ddr_ctrl_reg_write(    157,    0x02020202   );
    ddr_ctrl_reg_write(    158,    0x02020202   );
    ddr_ctrl_reg_write(    160,    0x01020202   );
    ddr_ctrl_reg_write(    161,    0x01010064   );
    ddr_ctrl_reg_write(    162,    0x01010101   );
    ddr_ctrl_reg_write(    163,    0x01010101   );
    ddr_ctrl_reg_write(    165,    0x00020101   );
    ddr_ctrl_reg_write(    166,    0x00000064   );
    ddr_ctrl_reg_write(    168,    0x000a0a00   );
    ddr_ctrl_reg_write(    170,    0x02000200   );
    ddr_ctrl_reg_write(    171,    0x02000200   );
    ddr_ctrl_reg_write(    175,    0x02000200   );
    ddr_ctrl_reg_write(    176,    0x02000200   );
    ddr_ctrl_reg_write(    180,    0x80000100   );
    ddr_ctrl_reg_write(    181,    0x04070303   );
    ddr_ctrl_reg_write(    182,    0x0000000a   );
    ddr_ctrl_reg_write(    185,    0x0010ffff   );
    ddr_ctrl_reg_write(    187,    0x0000000f   );
    ddr_ctrl_reg_write(    194,    0x00000204   );
    ddr_ctrl_reg_write(    205,    0x00000040   );

    // Fill registers specifically for DDR2 or DDR3
    if (ddr3){
        ddr_ctrl_reg_write(    0,    0x00000600     );
        ddr_ctrl_reg_write(    3,    0x0001046b     );
        ddr_ctrl_reg_write(    4,    0x00028b0b     );
        ddr_ctrl_reg_write(    5,    0x0c050c00     );
        ddr_ctrl_reg_write(    6,    0x04040405     );
        ddr_ctrl_reg_write(    7,    0x05040e14     );
        ddr_ctrl_reg_write(    8,    0x040e1404     );
        ddr_ctrl_reg_write(    9,    0x0c040405     );
        ddr_ctrl_reg_write(    10,    0x03005b68    );
        ddr_ctrl_reg_write(    11,    0x0c040404    );
        ddr_ctrl_reg_write(    12,    0x03005b68    );
        ddr_ctrl_reg_write(    13,    0x01000004    );
        ddr_ctrl_reg_write(    16,    0x03000200    );
        ddr_ctrl_reg_write(    17,    0x00000f0f    );
        ddr_ctrl_reg_write(    18,    0x05050000    );
        ddr_ctrl_reg_write(    20,    0x00007801    );
        ddr_ctrl_reg_write(    21,    0x00780a20    );
        ddr_ctrl_reg_write(    22,    0x00050a20    );
        ddr_ctrl_reg_write(    23,    0x00000300    );
        ddr_ctrl_reg_write(    24,    0x000a0003    );
        ddr_ctrl_reg_write(    25,    0x0000000a    );
        ddr_ctrl_reg_write(    27,    0x02000000    );
        ddr_ctrl_reg_write(    28,    0x0200005a    );
        ddr_ctrl_reg_write(    29,    0x0000005a    );
        ddr_ctrl_reg_write(    30,    0x05000001    );
        ddr_ctrl_reg_write(    31,    0x00050505    );
        ddr_ctrl_reg_write(    44,    0x00022000    );
        ddr_ctrl_reg_write(    45,    0x00000046    );
        ddr_ctrl_reg_write(    46,    0x00460210    );
        ddr_ctrl_reg_write(    48,    0x02100000    );
        ddr_ctrl_reg_write(    49,    0x00000046    );
        ddr_ctrl_reg_write(    50,    0x00460210    );
        ddr_ctrl_reg_write(    52,    0x02100000    );
        ddr_ctrl_reg_write(    53,    0x00000046    );
        ddr_ctrl_reg_write(    54,    0x00460210    );
        ddr_ctrl_reg_write(    56,    0x02100000    );
        ddr_ctrl_reg_write(    57,    0x00000046    );
        ddr_ctrl_reg_write(    58,    0x00460210    );
        ddr_ctrl_reg_write(    82,    0x01010001    );
        ddr_ctrl_reg_write(    108,    0x02040108   );
        ddr_ctrl_reg_write(    109,    0x08010402   );
        ddr_ctrl_reg_write(    110,    0x02020202   );
        ddr_ctrl_reg_write(    111,    0x01000201   );
        ddr_ctrl_reg_write(    128,    0x00212100   );
        ddr_ctrl_reg_write(    129,    0x21210001   );
        ddr_ctrl_reg_write(    133,    0x00012121   );
        ddr_ctrl_reg_write(    134,    0x00012121   );
        ddr_ctrl_reg_write(    137,    0x00212100   );
        ddr_ctrl_reg_write(    138,    0x21210001   );
        ddr_ctrl_reg_write(    142,    0x00012121   );
        ddr_ctrl_reg_write(    143,    0x00012121   );
        ddr_ctrl_reg_write(    146,    0x00212100   );
        ddr_ctrl_reg_write(    147,    0x21210001   );
        ddr_ctrl_reg_write(    169,    0x0a200000   );
        ddr_ctrl_reg_write(    172,    0x00000a20   );
        ddr_ctrl_reg_write(    173,    0x000032a0   );
        ddr_ctrl_reg_write(    174,    0x0a200505   );
        ddr_ctrl_reg_write(    177,    0x00000a20   );
        ddr_ctrl_reg_write(    178,    0x000032a0   );
        ddr_ctrl_reg_write(    179,    0x02020505   );
        ddr_ctrl_reg_write(    186,    0x16070303   );
        ddr_ctrl_reg_write(    202,    0x00000004   );
        ddr_ctrl_reg_write(    203,    0x00000004   );
        ddr_ctrl_reg_write(    206,    0x01040401   );
        ddr_ctrl_reg_write(    207,    0x00000001   );
    } else {
        ddr_ctrl_reg_write(    0,    0x00000400     );
        ddr_ctrl_reg_write(    3,    0x00000043     );
        ddr_ctrl_reg_write(    4,    0x000000a7     );
        ddr_ctrl_reg_write(    5,    0x0a040a02     );
        ddr_ctrl_reg_write(    6,    0x04020404     );
        ddr_ctrl_reg_write(    7,    0x05030e14     );
        ddr_ctrl_reg_write(    8,    0x030e1404     );
        ddr_ctrl_reg_write(    9,    0x04020305     );
        ddr_ctrl_reg_write(    10,    0x03005b26    );
        ddr_ctrl_reg_write(    11,    0x04020303    );
        ddr_ctrl_reg_write(    12,    0x03005b26    );
        ddr_ctrl_reg_write(    13,    0x01000003    );
        ddr_ctrl_reg_write(    16,    0x030000c8    );
        ddr_ctrl_reg_write(    17,    0x00860000    );
        ddr_ctrl_reg_write(    18,    0x05050086    );
        ddr_ctrl_reg_write(    20,    0x00002301    );
        ddr_ctrl_reg_write(    21,    0x00230a24    );
        ddr_ctrl_reg_write(    22,    0x00050a24    );
        ddr_ctrl_reg_write(    23,    0x00000200    );
        ddr_ctrl_reg_write(    24,    0x000a0002    );
        ddr_ctrl_reg_write(    25,    0x0002000a    );
        ddr_ctrl_reg_write(    26,    0x00020008    );
        ddr_ctrl_reg_write(    27,    0x00c80008    );
        ddr_ctrl_reg_write(    28,    0x00c80027    );
        ddr_ctrl_reg_write(    29,    0x00000027    );
        ddr_ctrl_reg_write(    30,    0x03000001    );
        ddr_ctrl_reg_write(    31,    0x00030303    );
        ddr_ctrl_reg_write(    44,    0x00085300    );
        ddr_ctrl_reg_write(    45,    0x00000004    );
        ddr_ctrl_reg_write(    46,    0x00040853    );
        ddr_ctrl_reg_write(    48,    0x08530000    );
        ddr_ctrl_reg_write(    49,    0x00000004    );
        ddr_ctrl_reg_write(    50,    0x00040853    );
        ddr_ctrl_reg_write(    52,    0x08530000    );
        ddr_ctrl_reg_write(    53,    0x00000004    );
        ddr_ctrl_reg_write(    54,    0x00040853    );
        ddr_ctrl_reg_write(    56,    0x08530000    );
        ddr_ctrl_reg_write(    57,    0x00000004    );
        ddr_ctrl_reg_write(    58,    0x00040853    );
        ddr_ctrl_reg_write(    82,    0x01030101    );
        ddr_ctrl_reg_write(    108,    0x02020101   );
        ddr_ctrl_reg_write(    109,    0x08080404   );
        ddr_ctrl_reg_write(    110,    0x03020200   );
        ddr_ctrl_reg_write(    111,    0x01000202   );
        ddr_ctrl_reg_write(    128,    0x001d1d00   );
        ddr_ctrl_reg_write(    129,    0x1d1d0001   );
        ddr_ctrl_reg_write(    133,    0x00011d1d   );
        ddr_ctrl_reg_write(    134,    0x00011d1d   );
        ddr_ctrl_reg_write(    137,    0x001d1d00   );
        ddr_ctrl_reg_write(    138,    0x1d1d0001   );
        ddr_ctrl_reg_write(    142,    0x00011d1d   );
        ddr_ctrl_reg_write(    143,    0x00011d1d   );
        ddr_ctrl_reg_write(    146,    0x001d1d00   );
        ddr_ctrl_reg_write(    147,    0x1d1d0001   );
        ddr_ctrl_reg_write(    169,    0x0a240000   );
        ddr_ctrl_reg_write(    172,    0x00000a24   );
        ddr_ctrl_reg_write(    173,    0x000032b4   );
        ddr_ctrl_reg_write(    174,    0x0a240404   );
        ddr_ctrl_reg_write(    177,    0x00000a24   );
        ddr_ctrl_reg_write(    178,    0x000032b4   );
        ddr_ctrl_reg_write(    179,    0x02020404   );
        ddr_ctrl_reg_write(    186,    0x15070303   );
        ddr_ctrl_reg_write(    202,    0x0001046b   );
        ddr_ctrl_reg_write(    203,    0x0001046b   );
        ddr_ctrl_reg_write(    206,    0x01030301   );
        ddr_ctrl_reg_write(    207,    0x00000001   );
    }
    
    // Start DDR
    *(unsigned int *)DDR_CTL_REGISTER_BASE |= 1;
    
    // Poll for DDR init done
    for(;;) {
        if (*(unsigned int *)0x18010164 & 0x200) {
            break;
        }
    }
}
.

//
// Wait until certain bits of the register become what we want
//
define /R wait_for_complete(reg, mask, value)
unsigned int reg;
unsigned int mask;
unsigned int value;
{
    for(;;) {
        if (((*(unsigned int *)reg) & mask) == value) {
            break;
        }
    }
}
.

//
// Generic MSPI translation (max 4 bytes tx)
//   Return: address to read data back (with +8 increment)
//
define /R unsigned int mspi_translation(tx_data, tx_len, rx_len)
unsigned int tx_data;
unsigned int tx_len;
unsigned int rx_len;
{
    int i;
    unsigned int dst;
    unsigned int total;
    
    // Fill TXRAM
    *(unsigned int *)(QSPI_REGISTER_BASE + 0x240) = tx_data & 0xff;
    *(unsigned int *)(QSPI_REGISTER_BASE + 0x248) = (tx_data >> 8) & 0xff;
    *(unsigned int *)(QSPI_REGISTER_BASE + 0x250) = (tx_data >> 16) & 0xff;
    *(unsigned int *)(QSPI_REGISTER_BASE + 0x258) = (tx_data >> 24) & 0xff;
    
    // Fill CDRAM
    total = tx_len + rx_len;
    dst = (QSPI_REGISTER_BASE + 0x340);
    for(i=0; i<total; i++, dst+=4) {
        *(unsigned int *)dst = 0x82;
        if (i == total - 1) {
            *(unsigned int *)dst = 0x02;
        }
    }
    
    // Queue pointers
    *(unsigned int *)(QSPI_REGISTER_BASE + 0x210) = 0;
    *(unsigned int *)(QSPI_REGISTER_BASE + 0x214) = total - 1;
    
    // Start it
    *(unsigned int *)(QSPI_REGISTER_BASE + 0x220) = 0x00;
    *(unsigned int *)(QSPI_REGISTER_BASE + 0x218) = 0xc0;
    
    // Wait for complete
    wait_for_complete((QSPI_REGISTER_BASE + 0x220), 1, 1);
    
    // Return the address of the first byte to read from
    return QSPI_REGISTER_BASE + 0x2c4 + 8 * tx_len;
}
.

//
// Get sflash block size by JEDEC ID
//
define /R int sflash_get_blocksize(dummy)
int dummy;
{
    if (JEDEC[0] == 0xc2) {
    
        /* Macronix */
        return 64 * 1024;
        
    } else if (JEDEC[0] == 0x20) {
    
        /* ST or Micron */
        if (JEDEC[1] == 0x20) {
            if (JEDEC[2] == 0x10 || JEDEC[2] == 0x11) {
                return 32 * 1024;
            }
            if (JEDEC[2] == 0x18) {
                return 256 * 1024;
            }
        }
        return 64 * 1024;
        
    } else if (JEDEC[0] == 0x01) {
    
        /* Spansion */
        if (JEDEC[1] == 0x02) {
            if (JEDEC[2] == 0x19 && JEDEC[3] == 0x4d && JEDEC[4] == 0x00) {
                return 256 * 1024;
            }
            if (JEDEC[2] == 0x20 && JEDEC[3] == 0x4d && JEDEC[4] == 0x00) {
                return 256 * 1024;
            }
            if (JEDEC[2] == 0x21 && JEDEC[3] == 0x4d && JEDEC[4] == 0x00) {
                return 256 * 1024;
            }
        } else if (JEDEC[1] == 0x20) {
            if (JEDEC[2] == 0x18 && JEDEC[3] == 0x03 && JEDEC[4] == 0x00) {
                return 256 * 1024;
            }
            if (JEDEC[2] == 0x18 && JEDEC[3] == 0x4d && JEDEC[4] == 0x00) {
                return 256 * 1024;
            }
        }
        return 64 * 1024;
    
    } else if (JEDEC[0] == 0xef) {
    
        /* Spansion or Winbond */
        return 64 * 1024;
        
    } else if (JEDEC[0] == 0x1f) {
    
        /* Atmel */
        if (JEDEC[1] == 0x66) {
            if (JEDEC[2] == 01) {
                return 32 * 1024;
            }
        }
        return 64 * 1024;
        
    } else if (JEDEC[0] == 0x1c) {
    
        /* EON */
        return 64 * 1024;
        
    } else if (JEDEC[0] == 0x89) {
    
        /* Intel/Numonyx */
        return 64 * 1024;
        
    } else if (JEDEC[0] == 0xBF) {
    
        /* SST: not uniform blocks, use 4KB sector erase */
        return 4 * 1024;
    }

    return 0;
}
.

//
// Erase blocks/sectors
//
define /R sflash_erase_block(fladdr, blksize, poll_fsr)
unsigned int fladdr;
unsigned int blksize;
unsigned int poll_fsr;
{
    unsigned int cmd;
    unsigned int rxaddr;
    
    // Write enable
    mspi_translation(0x06, 1, 0);

    // Erase one block/sector
    if (blksize == 4096) {
        // 4KB sector erase
        cmd = 0x20;
    } else {
        // Block erase
        cmd = 0xd8;
    }
    cmd |= ((fladdr >> 8) & 0xff00);
    cmd |= ((fladdr << 8) & 0xff0000);
    cmd |= ((fladdr << 24) & 0xff000000);
    mspi_translation(cmd, 4, 0);
    
    // Wait for erase complete
    for(;;) {
        rxaddr = mspi_translation(0x05, 1, 1);
        if ((*(unsigned int *)rxaddr & 1) == 0) {
            break;
        }
    }
    
    // Wait for erase/program controller ready (for some Micron flashes)
    if (poll_fsr) {
        for(;;) {
            rxaddr = mspi_translation(0x70, 1, 1);
            if ((*(unsigned int *)rxaddr & 0x80) == 0x80) {
                break;
            }
        }
    }
}
.

//
// Upload C code to write image to flash
//
define /R sflash_upload_program_code(addr)
unsigned int addr;
{
    $setmem /32 ${addr:x}+0x0000=0xe92d4ff8,0xe1a08003,0xe59f307c,0xe5830000$;
    $setmem /32 ${addr:x}+0x0010=0xe59f3078,0xe5832000,0xe2411001,0xe1a0a1a1$;
    $setmem /32 ${addr:x}+0x0020=0xe28aa001,0xe3a07000,0xe3a09006,0xe3a0b002$;
    $setmem /32 ${addr:x}+0x0030=0xe3a05005,0xe59f4058,0xe3a06070,0xe1a00009$;
    $setmem /32 ${addr:x}+0x0040=0xeb000014,0xe1a0000b,0xeb000012,0xe1a00005$;
    $setmem /32 ${addr:x}+0x0050=0xeb000010,0xe59432cc,0xe3130001,0x1afffffa$;
    $setmem /32 ${addr:x}+0x0060=0xe3580000,0x0a000004,0xe1a00006,0xeb000009$;
    $setmem /32 ${addr:x}+0x0070=0xe59432cc,0xe3130080,0x0afffffa,0xe2877001$;
    $setmem /32 ${addr:x}+0x0080=0xe15a0007,0x8affffec,0xe8bd8ff8,${addr:x}+0x1ac$;
    $setmem /32 ${addr:x}+0x0090=${addr:x}+0x1b0,${QSPI_REGISTER_BASE:x},0xe59f30f0,0xe5830240$;
    $setmem /32 ${addr:x}+0x00a0=0xe3500002,0x1a00001b,0xe59f30e4,0xe5932000$;
    $setmem /32 ${addr:x}+0x00b0=0xe1a01822,0xe20110ff,0xe59f30d0,0xe5831248$;
    $setmem /32 ${addr:x}+0x00c0=0xe1a01422,0xe20110ff,0xe5831250,0xe20220ff$;
    $setmem /32 ${addr:x}+0x00d0=0xe5832258,0xe59f20bc,0xe59f30bc,0xe59f10b0$;
    $setmem /32 ${addr:x}+0x00e0=0xe59fc0b8,0xe5930000,0xe5d00000,0xe4820008$;
    $setmem /32 ${addr:x}+0x00f0=0xe5930000,0xe2800001,0xe5830000,0xe5910000$;
    $setmem /32 ${addr:x}+0x0100=0xe2800001,0xe5810000,0xe152000c,0x1afffff4$;
    $setmem /32 ${addr:x}+0x0110=0xe3a0100b,0xea000005,0xe3500070,0x13500005$;
    $setmem /32 ${addr:x}+0x0120=0x13a01000,0x11a03001,0x1a000007,0xe3a01001$;
    $setmem /32 ${addr:x}+0x0130=0xe59f206c,0xe3a03000,0xe3a00082,0xe4820004$;
    $setmem /32 ${addr:x}+0x0140=0xe2833001,0xe1510003,0xcafffffb,0xe59f2054$;
    $setmem /32 ${addr:x}+0x0150=0xe0832002,0xe1a02102,0xe3a03002,0xe5823000$;
    $setmem /32 ${addr:x}+0x0160=0xe59f3028,0xe3a02000,0xe5832210,0xe5831214$;
    $setmem /32 ${addr:x}+0x0170=0xe5832220,0xe3a020c0,0xe5832218,0xe1a02003$;
    $setmem /32 ${addr:x}+0x0180=0xe5923220,0xe3130001,0x0afffffc,0xe12fff1e$;
    $setmem /32 ${addr:x}+0x0190=${QSPI_REGISTER_BASE:x},${addr:x}+0x1ac,${QSPI_REGISTER_BASE:x}+0x260,${addr:x}+0x1b0$;
    $setmem /32 ${addr:x}+0x01a0=${QSPI_REGISTER_BASE:x}+0x2a0,${QSPI_REGISTER_BASE:x}+0x340,(${QSPI_REGISTER_BASE:x}+0x340)/4$;
}
.

//
// Initialize QSPI and serial flash
//
define /R qspi_init(dummy)
int dummy;
{
    unsigned int rxaddr;

    // Switch to MSPI if not yet
    if (*(unsigned int *)(QSPI_REGISTER_BASE + 0x008) == 0) {
    
        // Wait for BSPI ready
        wait_for_complete((QSPI_REGISTER_BASE + 0x00c), 1, 0);
        
        // Switch to MSPI
        *(unsigned int *)(QSPI_REGISTER_BASE + 0x008) = 1;
    }
    
    // Basic MSPI configuration
    *(unsigned int *)(QSPI_REGISTER_BASE + 0x200) = 0x40;
    *(unsigned int *)(QSPI_REGISTER_BASE + 0x204) = 0xa3;
    
    // Read JEDEC ID
    rxaddr = mspi_translation(0x9f, 1, 5);
    JEDEC[0] = *(unsigned char *)rxaddr;
    JEDEC[1] = *(unsigned char *)(rxaddr + 8);
    JEDEC[2] = *(unsigned char *)(rxaddr + 16);
    JEDEC[3] = *(unsigned char *)(rxaddr + 24);
    JEDEC[4] = *(unsigned char *)(rxaddr + 32);
}
.

define /R verify_file(address, filesize)
unsigned int address;
unsigned int filesize;
{
    int r;
    
    $writefile ,raw ,nowarn "_tmpimg2.bin" =${address:x}..+${filesize:d}$;
    r = fopen(100, "_tmpimg.bin", "rb");
    if (r < 0) {
        error(3, "Cannot open file: %s!", "_tmpimg.bin");
        $failinc "Error opening file."$;
        return -1;
    }
    r = fopen(101, "_tmpimg2.bin", "rb");
    if (r < 0) {
        error(3, "Cannot open file: %s!", "_tmpimg2.bin");
        $failinc "Error opening file."$;
        return -1;
    }
    for(;;) {
        r = fgetc(100);
        if (r < 0) {
            break;
        }
        if (r != fgetc(101)) {
            break;
        }
    }
    fclose(100);
    fclose(101);
    if (r < 0) {
        return 1;
    }
    return 0;
}
.

define /R program_sflash()
{
    unsigned int start_offset;
    unsigned int filesize;
    unsigned int blksz;
    unsigned int poll_fsr;
    char filename[256];
    unsigned char buf[1024];
    unsigned int chunksize;
    unsigned int fladdr;
    unsigned int writesize;
    unsigned int i;
    int ddr_ok;
    int r;
    
    // Starting flash offset 
    start_offset = FLASH_START_OFFSET;
    
    // For safety on programming boot loaders, we erase the first block 
    // and perform programming after reset if boot code exists.
    ddr_ok = 0;
    if (FORCE_2_STEP_FOR_BOOT && start_offset == 0) {
        if (*(unsigned int *)FLASH_ADDRESS_BASE != 0xFFFFFFFF) {
            ddr_ok = -1; 
        }
    }
    
    if (ddr_ok != -1) {
        // Ask for the image to program
        r = prompt_file("Please select file to program", filename);
        if (r == 1) {
            $failinc "Cancelled."$;
            return;
        }
        
        if (!FORCE_2_STEP_FOR_BOOT && start_offset == 0) {
            // Disable L1 (I/D) and L2 caches for safety (TODO)
            *(unsigned int *)(L2CC_REGISTER_BASE + 0x100) = 0;
            $setreg @SCTLR=0x00C52078$;
        }
        
        if (USE_DDR) {
            // Check if DDR is already initialized
            if (*(unsigned int *)DDR_CTL_REGISTER_BASE & 1) {
                   
                // Note: perform MBIST tests could result in unexpected result.
                ddr_ok = 1;
                
            } else {
                // Basic DDR initialization at 333MHz
                $printf "### Initializing DDR."$;
                ddr_init(0);
                ddr_ok = 2;
            }
        } else {
            // Consider the RAM is ok if DDR is not used
            ddr_ok = 2;
        }

        // Copy file to memory if DDR is considered initialized
        if (ddr_ok > 0) {

            // Write the file to buffer (also calculate file size)
            $printf "### Checking file size."$;
            r = fopen(100, filename, "rb");
            if (r < 0) {
                error(3, "Cannot open file: %s!", filename);
                $failinc "Error opening file."$;
                return;
            }
            filesize = 0;
            r = fopen(101, "_tmpimg.bin", "wb");
            if (r < 0) {
                error(3, "Cannot creating file: %s!", "_tmpimg.bin");
                $failinc "Error creating file."$;
                return;
            }
            for(;;) {
                r = fread(buf, 1024, 1, 100);
                if (r <= 0) {
                    break;
                }
                filesize += r;
                r = fwrite(buf, r, 1, 101);
                if (r <= 0) {
                    break;
                }
            }
            fclose(100);
            fclose(101);
            $printf "### File size: %d (0x%x) bytes", filesize, filesize$;
            
            // Chunk size per write
            if (RAM_BUFFER_LENGTH - EXECUTION_RESERVED_SIZE > filesize) {
                chunksize = filesize;
                
                // Write file to memory
                $printf "### Writing the file to RAM."$;
                $readfile ,raw ,nowarn "_tmpimg.bin" =${RAM_BUFFER_START:x}+${EXECUTION_RESERVED_SIZE:x}$;
                
            } else {
                chunksize = RAM_BUFFER_LENGTH - EXECUTION_RESERVED_SIZE;
            }
            
            // Verify DDR if necessary
            if (ddr_ok < 2) {
                if (filesize == chunksize) {
                    $printf "### Verifying DDR with the file....."$;
                    r = verify_file(RAM_BUFFER_START + EXECUTION_RESERVED_SIZE, filesize);
                    if (r == 1) {
                        $printf "### Verified OK."$;
                        ddr_ok = 2;
                    } else if (r == 0) {
                        $printf "### Data unmatched! DDR is not correctly initialized!"$;
                    } else {
                        $host del /F /Q _tmpimg*.bin$;
                        return;
                    }
                } else {
                    // TODO: verify RAM even if it can't fit
                    ddr_ok = 2;
                }
            }
        }
    }

    // Initialize QSPI
    $printf "### Checking flash type."$;
    qspi_init(0);
    $printf "### Flash ID: %02x %02x %02x", JEDEC[0], JEDEC[1], JEDEC[2]$;
    blksz = sflash_get_blocksize(0);
    if (blksz == 0) {
        error(3, "Unrecognized/unsupported flash manufacturer: %x", JEDEC[0]);
        $failinc "Unrecognized/unsupported flash manufacturer."$;
        $host del /F /Q _tmpimg*.bin$;
        return;
    } else {
        $printf "### Flash block size: %u KB", blksz/1024$;
    }
    
    // Clear software protection bits if any
    if (JEDEC[0] == 0x1f || JEDEC[0] == 0x89 || JEDEC[0] == 0xbf) {
    
        // Write enable
        mspi_translation(0x06, 1, 0);
        
        // Write SR to 0
        mspi_translation(0x0001, 2, 0);
    }
    
    // For some Micron flashes, we need to poll for Flag Status Register
    if (JEDEC[0] == 0x20 && JEDEC[1] == 0xba) {
        poll_fsr = 1;
    } else {
        poll_fsr = 0;
    }
    
    // Erase the first block and quit the process if DDR isn't working
    if (ddr_ok < 2) {
        $printf "### Erasing the boot sector."$;
        sflash_erase_block(0, blksz, poll_fsr);
        $printf "### Boot sector erased. Please disconnect ICE, reset the target and try again."$;
        if (ddr_ok == -1) {
            error(3, "To avoid potential programming issues,\nboot sector have been erased for safety.\nPlease \n    1. Disconnect ICE\n    2. Reset the target\n    3. Connect ICE\n    4. Run this script again.", 0);
        } else {
            error(3, "DDR is not correctly configured!\nBoot sector is erased to avoid misconfiguration.\nPlease disconnect ICE, reset and try again.", 0);
        }
        if (ddr_ok > 0) {
            $host del /F /Q _tmpimg*.bin$;
        }
        return;
    }
    
    // Erase blocks
    filesize = ((filesize - 1) / 8 + 1) * 8;
    if (ERASE_BEFORE_PROGRAM) {
        // TODO: should avoid erasing the boot block again for FORCE_2_STEP_FOR_BOOT
        for(i = 0; i<filesize; i+=blksz) {
            fladdr = ((start_offset + i) / blksz) * blksz;
            $printf "### Erasing 0x%x bytes at offset 0x%x.", blksz, fladdr$;
            sflash_erase_block(fladdr, blksz, poll_fsr);
        }
    }
    
    // Upload binary code to memory for execution
    $printf "### Upload programming code to memory."$;
    sflash_upload_program_code(RAM_BUFFER_START);
    
    // Program it in one shot?
    if (filesize == chunksize) {
        // Set flash offset, file length and file buffer
        $printf "### Start programming....."$;
        $setreg @R0=${start_offset:x}$;
        $setreg @R1=${filesize:x}$;
        $setreg @R2=${RAM_BUFFER_START:x}+${EXECUTION_RESERVED_SIZE:x}$;
        $setreg @R3=${poll_fsr:d}$;
        
        // Set CPU registers and run C code
        $setreg @PC=${RAM_BUFFER_START:x}$;
        $setreg @SP=${RAM_BUFFER_START:x}+${EXECUTION_RESERVED_SIZE:x}-4$;
        $setreg @LR=${RAM_BUFFER_START:x}+${EXECUTION_RESERVED_SIZE:x}-4$;
        $go ${RAM_BUFFER_START:x}+${EXECUTION_RESERVED_SIZE:x}-4$;
    } else {
    
        // Open the image file in order to split it
        r = fopen(100, "_tmpimg.bin", "rb");
        if (r < 0) {
            error(3, "Cannot open file: %s!", "_tmpimg.bin");
            $failinc "Error opening file."$;
            return;
        }
    
        fladdr = start_offset;
        while(fladdr - start_offset < filesize) {
        
            // Bytes for this write
            writesize = 0;
        
            // Write the partial data into a file
            r = fopen(101, "_tmpimg3.bin", "wb");
            if (r < 0) {
                error(3, "Cannot open file: %s!", "_tmpimg3.bin");
                $failinc "Error opening file."$;
                return;
            }
            while(writesize < chunksize) {
                if (chunksize - writesize > 1024) {
                    i = 1024;
                } else {
                    i = chunksize - writesize;
                }
                
                r = fread(buf, i, 1, 100);
                if (r <= 0) {
                    break;
                }
                writesize += r;
                r = fwrite(buf, r, 1, 101);
                if (r <= 0) {
                    fclose(101);
                    error(3, "Cannot write file: %s!", "_tmpimg3.bin");
                    $failinc "Error writing file."$;
                    return;
                }
            }
            fclose(101);
            
            // Load the file into memory
            $readfile ,raw ,nowarn "_tmpimg3.bin" =${RAM_BUFFER_START:x}+${EXECUTION_RESERVED_SIZE:x}$;
            
            // Make it align with 8 bytes
            writesize = ((writesize - 1) / 8 + 1) * 8;
        
            // Set flash offset, file length and file buffer
            $printf "### Programming 0x%x bytes at offset 0x%x.....", writesize, fladdr$;
            $setreg @R0=${fladdr:x}$;
            $setreg @R1=${writesize:x}$;
            $setreg @R2=${RAM_BUFFER_START:x}+${EXECUTION_RESERVED_SIZE:x}$;
            $setreg @R3=${poll_fsr:d}$;
            
            // Set CPU registers and run C code
            $setreg @PC=${RAM_BUFFER_START:x}$;
            $setreg @SP=${RAM_BUFFER_START:x}+${EXECUTION_RESERVED_SIZE:x}-4$;
            $setreg @LR=${RAM_BUFFER_START:x}+${EXECUTION_RESERVED_SIZE:x}-4$;
            $go ${RAM_BUFFER_START:x}+${EXECUTION_RESERVED_SIZE:x}-4$;

            // Delete the file
            $host del /F /Q _tmpimg3.bin$;
            
            // Advance pointers
            fladdr += writesize;
        }
        
        fclose(100);
    }

    // Programming done
    $printf "### Programming done."$;
    
    // Switch back to BSPI
    *(unsigned int *)(QSPI_REGISTER_BASE + 0x008) = 0;
    
    // Start to verify
    $printf "### Verifying....."$;
    
    // Invalidate bspi buffer
    *(unsigned int *)(QSPI_REGISTER_BASE + 0x018) = 1;
    *(unsigned int *)(QSPI_REGISTER_BASE + 0x020) = 1;
    
    // Verify using the file
    r = verify_file(FLASH_ADDRESS_BASE + start_offset, filesize);
    if (r == 1) {
        $printf "### Verified OK."$;
        $printf "### Programming complete. Please disconnect ICE and reset the device."$;
    } else if (r == 0) {
        $printf "### Data on flash unmatched! Programming may not be successfully!"$;
        error(3, "Programming may not be successfully!\nPlease disconnect ICE, reset and try again.", 0);
    }

    // Clean up
    $host del /F /Q _tmpimg*.bin$;
}
.

// Start it
macro program_sflash()
macro sflash_delete_symbols(0)
