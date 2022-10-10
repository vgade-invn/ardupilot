/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  simple bitmask class
 */

#pragma once

#include <stdint.h>
#include <string.h>
#include <AP_InternalError/AP_InternalError.h>

/*
  underlying Bitmask class that takes a pre-allocated array of bits
 */
class _Bitmask {
public:
    _Bitmask(uint16_t num_bits, uint32_t *bits);

    _Bitmask(const _Bitmask &other) = delete;

    _Bitmask &operator=(const _Bitmask &other);

    bool operator==(const _Bitmask&other);

    bool operator!=(const _Bitmask&other);


    // set given bitnumber
    void set(uint16_t bit);

    // set all bits
    void setall(void);

    // clear given bitnumber
    void clear(uint16_t bit);

    // set given bitnumber to on/off
    void setonoff(uint16_t bit, bool onoff);

    // clear all bits
    void clearall(void);

    // return true if given bitnumber is set
    bool get(uint16_t bit) const;

    // return true if all bits are clear
    bool empty(void) const;

    // return number of bits set
    uint16_t count() const;

    // return first bit set, or -1 if none set
    int16_t first_set() const;

    // return number of bits available
    uint16_t size() const {
        return numbits;
    }

private:
    uint16_t numbits;
    uint16_t numwords;
    uint32_t *bits;
};

/*
  template class for a specific size with allocation in the class
 */
template<uint16_t num_bits>
class Bitmask : public _Bitmask {
public:
    Bitmask() : _Bitmask(num_bits, bits) {}

private:
    uint32_t bits[(num_bits+31)/32];
};
