// Copyright (c) 2023, Wei Jian, Co.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <iostream>
#include <unistd.h>
#include "protocol.hpp"
#include "udp.hpp"
#include "utils.hpp"

/**
 *
 * Vendor's original code.
 * We will reimplement it.
 *
 */

namespace wlr
{

Protocol::Protocol(UDPTrans *trans)
  : trans_(trans)
  {
    memset(&m_sdata, 0, sizeof(m_sdata));
    pre_fidx = 0;

    scan_ = trans_->get_scan();
    range_min_ = scan_->range_min;
    range_max_ = scan_->range_max;
  }

  bool Protocol::dataProcess(unsigned char *data, const int reclen)
  {
    if (reclen > MAX_LENGTH_DATA_PROCESS)
    {
      return false;
    }

    if (m_sdata.m_u32in + reclen > MAX_LENGTH_DATA_PROCESS)
    {
      memset(&m_sdata, 0, sizeof(m_sdata));
      return false;
    }
    memcpy(&m_sdata.m_acdata[m_sdata.m_u32in], data, reclen * sizeof(char));
    m_sdata.m_u32in += reclen;
    while (m_sdata.m_u32out < m_sdata.m_u32in)
    {
      if ((m_sdata.m_acdata[m_sdata.m_u32out] == 0x02 && m_sdata.m_acdata[m_sdata.m_u32out + 1] == 0x02 &&
           m_sdata.m_acdata[m_sdata.m_u32out + 2] == 0x02 && m_sdata.m_acdata[m_sdata.m_u32out + 3] == 0x02) ||
          (m_sdata.m_acdata[m_sdata.m_u32out] == 0xFF && m_sdata.m_acdata[m_sdata.m_u32out + 1] == 0xAA))
      {
        unsigned l_u32reallen = 0;
        if (m_sdata.m_acdata[m_sdata.m_u32out] == 0x02)
        {
          l_u32reallen = (m_sdata.m_acdata[m_sdata.m_u32out + 4] << 24) |
                         (m_sdata.m_acdata[m_sdata.m_u32out + 5] << 16) |
                         (m_sdata.m_acdata[m_sdata.m_u32out + 6] << 8) |
                         (m_sdata.m_acdata[m_sdata.m_u32out + 7] << 0);
          l_u32reallen = l_u32reallen + 9;
        }
        else
        {
          l_u32reallen = (m_sdata.m_acdata[m_sdata.m_u32out + 2] << 8) |
                         (m_sdata.m_acdata[m_sdata.m_u32out + 3]);
          l_u32reallen = l_u32reallen + 4;
        }

        if (l_u32reallen <= (m_sdata.m_u32in - m_sdata.m_u32out + 1))
        {
          if (OnRecvProcess(&m_sdata.m_acdata[m_sdata.m_u32out], l_u32reallen))
          {
            m_sdata.m_u32out += l_u32reallen;
          }
          else
          {
            cout << "continuous frame" << endl;
            int i;
            for (i = 1; i <= (int)l_u32reallen; i++)
            {
              if (((m_sdata.m_acdata[m_sdata.m_u32out + i] == 0x02) &&
                   (m_sdata.m_acdata[m_sdata.m_u32out + i + 1] == 0x02) &&
                   (m_sdata.m_acdata[m_sdata.m_u32out + i + 2] == 0x02) &&
                   (m_sdata.m_acdata[m_sdata.m_u32out + i + 3] == 0x02)) ||
                  (m_sdata.m_acdata[m_sdata.m_u32out] == 0xFF && m_sdata.m_acdata[m_sdata.m_u32out + 1] == 0xAA))
              {
                m_sdata.m_u32out += i;
                break;
              }
              if (i == (int)l_u32reallen)
              {
                m_sdata.m_u32out += l_u32reallen;
              }
            }
          }
        }
        else if (l_u32reallen >= MAX_LENGTH_DATA_PROCESS)
        {
          cout << "l_u32reallen >= MAX_LENGTH_DATA_PROCESS" << endl;
          cout << "reallen: " << l_u32reallen << endl;
          memset(&m_sdata, 0, sizeof(m_sdata));
        }
        else
        {
          break;
        }
      }
      else
      {
        m_sdata.m_u32out++;
      }
    } //end while(m_sdata.m_u32out < m_sdata.m_u32in)

    if (m_sdata.m_u32out >= m_sdata.m_u32in)
    {
      memset(&m_sdata, 0, sizeof(m_sdata));
    }
    return true;
  }

  bool Protocol::OnRecvProcess(unsigned char *data, int len)
  {
    if (len > 0)
    {
      if (checkXor(data, len))
      {
        protocl(data, len);
      }
    }
    else
    {
      return false;
    }
    return true;
  }

  bool Protocol::protocl(unsigned char *data, const int len)
  {
    (void)len;
    //Frame header: "02 02 02 02"
    if (data[0] == 0x02 && data[1] == 0x02 && data[2] == 0x02 && data[3] == 0x02)
    {
      //Instructions: "73 53 4E 20 4C 4D 44 73 63 61 6E 64 61 74 61 20"
      if ((data[8] == 0x73 && data[9] == 0x52) || (data[8] == 0x73 && data[9] == 0x53)) //command type:0x73 0x52/0X53
      {
        static int sindex;
	//Point count. 16 bits.
	//int pcount = (data[83] << 8) + data[84];
        //cout << "Point count: " << pcount << endl;
	//Package number. 16 bits.
        int pindex = (data[50] << 8) + data[51];
	//Loop number. 32 bits.
        unsigned int findex = (data[46] << 24) + (data[47] << 16) + (data[48] << 8) + data[49];
        //cout << "findex: " << findex << endl;
        //cout << "pindex: " << pindex << endl;
        if (pindex == 1)
        {
          //clock_gettime(CLOCK_MONOTONIC, &start);
          sindex = 0;
          pre_fidx = findex;
	  //Distance, 16 bits per point. Odd number package contains 405 points. That is to say 810 bytes.
          for (int j = 0; j < 810; j = j + 2)
          {
            scandata[sindex] = (((unsigned char)data[85 + j]) << 8) + ((unsigned char)data[86 + j]);
            scandata[sindex] /= 1000.0;
            //if (scandata[sindex] > scan.range_max || scandata[sindex] < scan.range_min || scandata[sindex] == 0)
            if (scandata[sindex] > range_max_ || (int)scandata[sindex] < (int)range_min_)
            {
              scandata[sindex] = NAN;
            }
            sindex++;
          }
        }
        else if (pindex == 2)
        {
          if (pre_fidx == findex)
          {
	    //Distance, 16 bits per point. Even number package contains 406 points. That is to say 812 bytes.
            for (int j = 0; j < 812; j = j + 2)
            {
              scandata[sindex] = (((unsigned char)data[85 + j]) << 8) + ((unsigned char)data[86 + j]);
              scandata[sindex] /= 1000.0;
              //if (scandata[sindex] > scan.range_max || scandata[sindex] < scan.range_min || scandata[sindex] == 0)
              if (scandata[sindex] > range_max_ || (int)scandata[sindex] < (int)range_min_)
              {
                scandata[sindex] = NAN;
              }
              sindex++;
            }
          }
          else
          {
            sindex = 0;
            return false;
          }
        }
        else if (pindex == 3)
        {
          sindex = 0;
          if (pre_fidx == findex)
          {
            //Intensity, 16 bits per point. Odd number package contains 405 points. That is to say 810 bytes.
            for (int j = 0; j < 810; j = j + 2)
            {
              scaninden[sindex] = (((unsigned char)data[85 + j]) << 8) + ((unsigned char)data[86 + j]);
              sindex++;
            }
          }
          else
          {
            sindex = 0;
            return false;
          }
        }
        else if (pindex == 4)
        {
          if (pre_fidx == findex)
          {
             //Intensity, 16 bits per point. Even number package contains 406 points. That is to say 812 bytes.
            for (int j = 0; j < 812; j = j + 2)
            {
              scaninden[sindex] = (((unsigned char)data[85 + j]) << 8) + ((unsigned char)data[86 + j]);
              sindex++;
            }
	    trans_->set_scan_data(findex, scandata, scaninden);
          }
          else
          {
            sindex = 0;
            return false;
          }
        }
        return true;
      }
      else
      {
        return false;
      }
    }
    else if (data[0] == 0xFF && data[1] == 0xAA)
    {
      if ((data[22] == 0x06 && data[23] == 0x08)) //command GetMAC
      {
/*
 *      Do not support.
 */
        return true;
      }
      else
      {
        return false;
      }
    }
    else
    {
      return false;
    }
  }

  bool Protocol::checkXor(unsigned char *recvbuf, int recvlen)
  {
    int i = 0;
    unsigned char check = 0;
    unsigned char *p = recvbuf;
    int len;

    if (*p == (char)0x02)
    {
      p = p + 8;
      len = recvlen - 9;
      for (i = 0; i < len; i++)
      {
        check ^= *p++;
      }
    }
    else
    {
      p = p + 2;
      len = recvlen - 6;
      for (i = 0; i < len; i++)
      {
        check ^= *p++;
      }
      p++;
    }

    if (check == *p)
    {
      return true;
    }
    else
      return false;
  }

} //name space
