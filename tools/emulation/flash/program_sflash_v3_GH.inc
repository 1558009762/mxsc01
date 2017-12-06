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
ADD unsigned int USE_DDR                        = 0 // Not applicable for GH
ADD unsigned int RAM_BUFFER_START               = 0x02000000
ADD unsigned int RAM_BUFFER_LENGTH              = 32*1024
ADD unsigned int FLASH_ADDRESS_BASE             = 0xF0000000
ADD unsigned int QSPI_REGISTER_BASE             = 0x18047000
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

//
// DDR initialization
// 
define /R ddr_init(dummy)
int dummy;
{
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
    
    // In case it's running in big endian
    $setreg @CPSR=0x00000113$;
    
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
            // ddr init is only used for NS now
            // // Check if DDR is already initialized
            // if (*(unsigned int *)DDR_CTL_REGISTER_BASE & 1) {
            //        
            //     // Note: perform MBIST tests could result in unexpected result.
            //     ddr_ok = 1;
            //     
            // } else {
            //     // Basic DDR initialization at 333MHz
            //     $printf "### Initializing DDR."$;
            //     ddr_init(0);
            //     ddr_ok = 2;
            // }
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
