/*
 * Copyright 2018 <Admobilize>
 * MATRIX Labs  [http://creator.matrix.one]
 * This file is part of MATRIX Creator HAL
 *
 * MATRIX Creator HAL is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CPP_DRIVER_BUS_KERNEL_H_
#define CPP_DRIVER_BUS_KERNEL_H_

#include <stdint.h>
#include <mutex>
#include <string>
#include "./bus.h"

namespace matrix_hal {

class BusKernel : public Bus {
 public:
  BusKernel();
  virtual ~BusKernel();

  virtual bool Init(std::string device_name);

  virtual bool Write(uint16_t add, unsigned char *data, int length);

  virtual bool Read(uint16_t add, unsigned char *data, int length);

  virtual void Close();

 private:
  int regmap_fd_;
};
};      // namespace matrix_hal
#endif  // CPP_DRIVER_BUS_KERNEL_H_
