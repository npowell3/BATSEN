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
#include "lr-wpan-sinr-tag.h"
#include <ns3/integer.h>

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (LrWpanSinrTag);

TypeId
LrWpanSinrTag::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::LrWpanSinrTag")
    .SetParent<Tag> ()
    .SetGroupName ("LrWpan")
    .AddConstructor<LrWpanSinrTag> ()
    .AddAttribute ("Sinr", "The sinr of the last packet received",
                   IntegerValue (0),
                   MakeIntegerAccessor (&LrWpanSinrTag::Get),
                   MakeIntegerChecker<uint8_t> ())
  ;
  return tid;
}

TypeId
LrWpanSinrTag::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

LrWpanSinrTag::LrWpanSinrTag (void)
  : m_sinr (0.0)
{
}

LrWpanSinrTag::LrWpanSinrTag (double sinr)
  : m_sinr (sinr)
{
}

uint32_t
LrWpanSinrTag::GetSerializedSize (void) const
{
  return sizeof (double);
}

void
LrWpanSinrTag::Serialize (TagBuffer i) const
{
  i.WriteU8 (m_sinr);
}

void
LrWpanSinrTag::Deserialize (TagBuffer i)
{
  m_sinr = i.ReadU8 ();
}

void
LrWpanSinrTag::Print (std::ostream &os) const
{
  os << "Sinr = " << m_sinr;
}

void
LrWpanSinrTag::Set (double sinr)
{
  m_sinr = sinr;
}

double
LrWpanSinrTag::Get (void) const
{
  return m_sinr;
}

}
