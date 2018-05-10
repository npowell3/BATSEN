//
//  lr-wpan-sinr-tag.h
//  
//
//  Created by Nelson Powell on 6/30/17.
//
//  Copied and reworked from LQI Tag file to support passing SINR values to
//  SensorNode MAC algorithms.
//
//
/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2013 Fraunhofer FKIE
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
 * Author:
 *  Sascha Alexander Jopen <jopen@cs.uni-bonn.de>
 */

#ifndef lr_wpan_sinr_tag_h
#define lr_wpan_sinr_tag_h

#include <ns3/tag.h>

namespace ns3 {
  
  class LrWpanSinrTag : public Tag
  {
  public:
    /**
     * Get the type ID.
     *
     * \return the object TypeId
     */
    static TypeId GetTypeId (void);
    
    virtual TypeId GetInstanceTypeId (void) const;
    
    /**
     * Create a LrWpanSinrTag with the default SINR 0..0
     */
    LrWpanSinrTag (void);
    
    /**
     * Create a LrWpanSinrTag with the given SINR value.
     */
    LrWpanSinrTag (double sinr);
    
    virtual uint32_t GetSerializedSize (void) const;
    virtual void Serialize (TagBuffer i) const;
    virtual void Deserialize (TagBuffer i);
    virtual void Print (std::ostream &os) const;
    
    /**
     * Set the SINR to the given value.
     *
     * \param sinr the value of the SINR to set
     */
    void Set (double sinr);
    
    /**
     * Get the SINR value.
     *
     * \return the SINR value
     */
    double Get (void) const;
  private:
    /**
     * The current SINR value of the tag.
     */
    double m_sinr;
  };
  
  
}

#endif /* lr_wpan_sinr_tag_h */
