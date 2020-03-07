/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 CTTC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Nicola Baldo <nbaldo@cttc.es>
 * Modified by Marco Miozzo <mmiozzo@cttc.es> (add data and ctrl diversity)
 */

#ifndef MMWAVE_SPECTRUM_SIGNAL_PARAMETERS_H
#define MMWAVE_SPECTRUM_SIGNAL_PARAMETERS_H


#include <ns3/spectrum-signal-parameters.h>

namespace ns3 {

class PacketBurst;
class MmWaveControlMessage;

/**
 * \ingroup mmwave
 *
 * Signal parameters for mmwave
 */
struct mmwaveSpectrumSignalParameters : public SpectrumSignalParameters
{

  // inherited from SpectrumSignalParameters
  virtual Ptr<SpectrumSignalParameters> Copy ();

  /**
   * default constructor
   */
  mmwaveSpectrumSignalParameters ();

  /**
   * copy constructor
   */
  mmwaveSpectrumSignalParameters (const mmwaveSpectrumSignalParameters& p);

  Ptr<PacketBurst> packetBurst;

};



struct MmwaveSpectrumSignalParametersDataFrame : public SpectrumSignalParameters
{
  
  // inherited from SpectrumSignalParameters
  virtual Ptr<SpectrumSignalParameters> Copy ();
  
  /**
  * default constructor
  */
  MmwaveSpectrumSignalParametersDataFrame ();
  
  /**
  * copy constructor
  */
  MmwaveSpectrumSignalParametersDataFrame (const MmwaveSpectrumSignalParametersDataFrame& p);
  
  Ptr<PacketBurst> packetBurst;

  std::list<Ptr<MmWaveControlMessage> > ctrlMsgList;
  
  uint16_t cellId;

  uint8_t slotInd;
};


struct MmWaveSpectrumSignalParametersDlCtrlFrame : public SpectrumSignalParameters
{
  
  // inherited from SpectrumSignalParameters
  virtual Ptr<SpectrumSignalParameters> Copy ();
  
  /**
  * default constructor
  */
  MmWaveSpectrumSignalParametersDlCtrlFrame ();
  
  /**
  * copy constructor
  */
  MmWaveSpectrumSignalParametersDlCtrlFrame (const MmWaveSpectrumSignalParametersDlCtrlFrame& p);
  

  std::list<Ptr<MmWaveControlMessage> > ctrlMsgList;

  bool pss;
  uint16_t cellId;
  
  //***Acrescentei os dois membros abaixo:
  
  Ptr<PacketBurst> packetBurst;
  
  uint8_t slotInd;
  
  //****
};


}  // namespace ns3


#endif /* MMWAVE_SPECTRUM_SIGNAL_PARAMETERS_H */
