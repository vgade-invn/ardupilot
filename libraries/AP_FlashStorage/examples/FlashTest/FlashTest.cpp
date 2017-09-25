//
// Unit tests for the AP_Math rotations code
//

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

//#pragma GCC optimize("O0")

#include <AP_FlashStorage/AP_FlashStorage.h>
#include <stdio.h>

#define USE_HAL_PX4_FLASH 1

#if USE_HAL_PX4_FLASH
#include <nuttx/progmem.h>
static const uint32_t _flash_page = 22;
#endif

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

class FlashTest : public AP_HAL::HAL::Callbacks {
public:
    // HAL::Callbacks implementation.
    void setup() override;
    void loop() override;

private:
    static const uint32_t flash_sector_size = 128U * 1024U;

    uint8_t mem_buffer[AP_FlashStorage::storage_size];
    uint8_t mem_mirror[AP_FlashStorage::storage_size];

#if !USE_HAL_PX4_FLASH
    // flash buffer
    uint8_t flash[2][flash_sector_size];
#endif

    bool flash_write(uint8_t sector, uint32_t offset, const uint8_t *data, uint16_t length);
    bool flash_read(uint8_t sector, uint32_t offset, uint8_t *data, uint16_t length);
    bool flash_erase(uint8_t sector);
    bool flash_erase_ok(void);
    
    AP_FlashStorage storage{mem_buffer,
            flash_sector_size,
            FUNCTOR_BIND_MEMBER(&FlashTest::flash_write, bool, uint8_t, uint32_t, const uint8_t *, uint16_t),
            FUNCTOR_BIND_MEMBER(&FlashTest::flash_read, bool, uint8_t, uint32_t, uint8_t *, uint16_t),
            FUNCTOR_BIND_MEMBER(&FlashTest::flash_erase, bool, uint8_t),
            FUNCTOR_BIND_MEMBER(&FlashTest::flash_erase_ok, bool)};

    // write to storage and mem_mirror
    void write(uint16_t offset, const uint8_t *data, uint16_t length);

    bool erase_ok;
    uint32_t _offset=0;
    uint32_t _count=0;
};

#if USE_HAL_PX4_FLASH

#define SETLINE do { _flash_line=__LINE__; _flash_line2=__LINE__; } while (0)

static uint32_t _flash_line;
static uint32_t _flash_offset;
static uint32_t _flash_sector;
static const uint8_t *_flash_data;
static uint32_t _flash_length;
static uint32_t _flash_line2;


/*
  callback to write data to flash
 */
bool FlashTest::flash_write(uint8_t sector, uint32_t offset, const uint8_t *data, uint16_t length)
{
    SETLINE;
    _flash_sector = sector;
    _flash_offset = offset;
    _flash_data = data;
    _flash_length = length;
    
    size_t base_address = up_progmem_getaddress(_flash_page+sector);
    const uint8_t *b = ((const uint8_t *)base_address)+offset;
    for (uint32_t i=0; i<length; i++) {
        if (data[i] & ~b[i]) {
            AP_HAL::panic("invalid flash write\n");
            SETLINE;
            return false;
        }
    }
    SETLINE;
    bool ret = up_progmem_write(base_address+offset, data, length) == length;
    SETLINE;
    if (memcmp(b, data, length) != 0) {
        AP_HAL::panic("flash write failed\n");
        SETLINE;
        return false;
    }
    SETLINE;
    return ret;
}

/*
  callback to read data from flash
 */
bool FlashTest::flash_read(uint8_t sector, uint32_t offset, uint8_t *data, uint16_t length)
{
    SETLINE;
    size_t base_address = up_progmem_getaddress(_flash_page+sector);
    SETLINE;
    const uint8_t *b = ((const uint8_t *)base_address)+offset;
    SETLINE;
    memcpy(data, b, length);
    SETLINE;
    return true;
}

/*
  callback to erase flash sector
 */
bool FlashTest::flash_erase(uint8_t sector)
{
    SETLINE;
    bool ret = up_progmem_erasepage(_flash_page+sector) > 0;
    SETLINE;
    return ret;
}
#else

bool FlashTest::flash_write(uint8_t sector, uint32_t offset, const uint8_t *data, uint16_t length)
{
    if (sector > 1) {
        AP_HAL::panic("FATAL: write to sector %u\n", (unsigned)sector);
    }
    if (offset + length > flash_sector_size) {
        AP_HAL::panic("FATAL: write to sector %u at offset %u length %u\n",
                      (unsigned)sector,
                      (unsigned)offset,
                      (unsigned)length);
    }
    uint8_t *b = &flash[sector][offset];
    for (uint32_t i=0; i<length; i++) {
        if (data[i] & ~b[i]) {
            AP_HAL::panic("invalid flash write\n");
            return false;
        }
    }
    for (uint16_t i=0; i<length; i++) {
        b[i] &= data[i];
    }
    return true;
}

bool FlashTest::flash_read(uint8_t sector, uint32_t offset, uint8_t *data, uint16_t length)
{
    if (sector > 1) {
        AP_HAL::panic("FATAL: read from sector %u\n", (unsigned)sector);
    }
    if (offset + length > flash_sector_size) {
        AP_HAL::panic("FATAL: read from sector %u at offset %u length %u\n",
                      (unsigned)sector,
                      (unsigned)offset,
                      (unsigned)length);
    }
    memcpy(data, &flash[sector][offset], length);
    return true;
}

bool FlashTest::flash_erase(uint8_t sector)
{
    if (sector > 1) {
        AP_HAL::panic("FATAL: erase sector %u\n", (unsigned)sector);
    }
    memset(&flash[sector][0], 0xFF, flash_sector_size);
    return true;
}

#endif // USE_HAL_PX4_FLASH

bool FlashTest::flash_erase_ok(void)
{
    return erase_ok;
}

void FlashTest::write(uint16_t offset, const uint8_t *data, uint16_t length)
{
    memcpy(&mem_mirror[offset], data, length);
    memcpy(&mem_buffer[offset], data, length);
    if (!storage.write(offset, length)) {
        if (erase_ok) {
            printf("Failed to write at %u for %u\n", offset, length);
        }
    }
}

/*
 * test flash storage
 */
void FlashTest::setup(void)
{
    flash_erase(0);
    flash_erase(1);
    hal.console->printf("AP_FlashStorage test\n");

    if (!storage.init()) {
        AP_HAL::panic("Failed first init()");
    }

#if 0
    // fill with 10k random writes
    for (uint32_t i=0; i<5000000; i++) {
        uint16_t ofs = get_random16() % sizeof(mem_buffer);
        uint16_t length = get_random16() & 0x1F;
        length = MIN(length, sizeof(mem_buffer) - ofs);
        uint8_t data[length];
        for (uint8_t j=0; j<length; j++) {
            data[j] = get_random16() & 0xFF;
        }

        erase_ok = (i % 1000 == 0);
        write(ofs, data, length);

        if (erase_ok) {
            if (memcmp(mem_buffer, mem_mirror, sizeof(mem_buffer)) != 0) {
                AP_HAL::panic("FATAL: data mis-match at i=%u", i);
            }
        }
    }

    // force final write to allow for flush with erase_ok
    erase_ok = true;
    uint8_t b = 42;
    write(37, &b, 1);
    
    if (memcmp(mem_buffer, mem_mirror, sizeof(mem_buffer)) != 0) {
        AP_HAL::panic("FATAL: data mis-match before re-init");
    }
    
    // re-init
    printf("re-init\n");
    memset(mem_buffer, 0, sizeof(mem_buffer));
    if (!storage.init()) {
        AP_HAL::panic("Failed second init()");
    }

    if (memcmp(mem_buffer, mem_mirror, sizeof(mem_buffer)) != 0) {
        AP_HAL::panic("FATAL: data mis-match");
    }
    AP_HAL::panic("TEST PASSED");
#endif
}

void FlashTest::loop(void)
{
    erase_ok = true;
    if (_count % 4096 == 0) {
        printf("write %u at %u\n", (unsigned)_count, (unsigned)_offset);
    }
    for (uint32_t i=0; i<256; i++) {
        write(_offset, (const uint8_t *)&_count, sizeof(_count));
        _offset = (_offset + 4) % HAL_STORAGE_SIZE;
        _count++;
    }
    hal.scheduler->delay(10);
}

FlashTest flashtest;

AP_HAL_MAIN_CALLBACKS(&flashtest);
