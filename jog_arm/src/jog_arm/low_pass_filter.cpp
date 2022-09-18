///////////////////////////////////////////////////////////////////////////////
//      Title     : jog_arm_server.h
//      Project   : jog_arm
//      Created   : 3/9/2017
//      Author    : Brian O'Neil, Blake Anderson, Andy Zelenak
//
// BSD 3-Clause License
//
// Copyright (c) 2018, Los Alamos National Security, LLC
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////////

#include <jog_arm/low_pass_filter.h>

using namespace jog_arm;

LowPassFilter::LowPassFilter(const double low_pass_filter_coeff)
{
  filter_coeff_ = low_pass_filter_coeff;
}

void LowPassFilter::reset(const double data)
{
  prev_msrmts_[0] = data;
  prev_msrmts_[1] = data;
  prev_msrmts_[2] = data;

  prev_filtered_msrmts_[0] = data;
  prev_filtered_msrmts_[1] = data;
}

double LowPassFilter::filter(const double new_msrmt)
{
  // Push in the new measurement
  prev_msrmts_[2] = prev_msrmts_[1];
  prev_msrmts_[1] = prev_msrmts_[0];
  prev_msrmts_[0] = new_msrmt;

  double new_filtered_msrmt = (1 / (1 + filter_coeff_ * filter_coeff_ + 1.414 * filter_coeff_)) *
                              (prev_msrmts_[2] + 2 * prev_msrmts_[1] + prev_msrmts_[0] -
                               (filter_coeff_ * filter_coeff_ - 1.414 * filter_coeff_ + 1) * prev_filtered_msrmts_[1] -
                               (-2 * filter_coeff_ * filter_coeff_ + 2) * prev_filtered_msrmts_[0]);

  // Store the new filtered measurement
  prev_filtered_msrmts_[1] = prev_filtered_msrmts_[0];
  prev_filtered_msrmts_[0] = new_filtered_msrmt;

  return new_filtered_msrmt;
}
