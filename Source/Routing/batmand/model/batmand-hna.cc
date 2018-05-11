/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

/*
 * Copyright (c) 2016 Rochester Institute of Technology
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
 * Author: Nelson Powell <nhp8080@rit.edu>
 *
 * NOTE:
 *
 * Code based on the BATMAND module and modified for BATMAND-0.3.2
 * implementation.  BATMAND was the predecessor for BATMAN and has many
 * similar features.  Plus, modifying the BATMAND module reduces the
 * effort required to write a module from scratch.
 *
 * The BATMAN module is based on the IETF draft found at
 * https://tools.ietf.org/html/draft-openmesh-b-a-t-m-a-n-00 and the
 * BATMAN-0.3.2 code base downloadable from
 * https://www.open-mesh.org/projects/open-mesh/wiki/Download
 *
 *
 */

///
/// \brief Implementation of BATMAND agent and related classes.
///
/// This is the main file of this software because %BATMAND's behavior is
/// implemented here.
///

#include "batmand-routing-protocol.h"
#include "ns3/ipv4-address.h"
#include "ns3/log.h"
#include "ns3/names.h"

#include <arpa/inet.h>

namespace ns3 {
  
NS_LOG_COMPONENT_DEFINE ("BatmanHna");

namespace batmand {

int RoutingProtocol::compare_hna(void *data1, void *data2)
{
  return (memcmp(data1, data2, 5) == 0 ? 1 : 0);
}

int RoutingProtocol::choose_hna(void *data, int32_t size)
{
  unsigned char *key = (unsigned char *)data;
  uint32_t hash = 0;
  size_t i;

  for (i = 0; i < 5; i++) {
    hash += key[i];
    hash += (hash << 10);
    hash ^= (hash >> 6);
  }

  hash += (hash << 3);
  hash ^= (hash >> 11);
  hash += (hash << 15);

  return (hash % size);
}

void RoutingProtocol::hna_init(void)
{
  /* hna global */
  m_hnaGlobalHash = hash_new( 128 );

  if (m_hnaGlobalHash == NULL) {
    NS_LOG_DEBUG( "Error - Could not create m_hnaGlobalHash (out of memory?)");
    exit(EXIT_FAILURE);
  }
}

  
  
void RoutingProtocol::HNAGlobalAdd(struct orig_node *orig_node, unsigned char *new_hna, int16_t new_hna_len)
{
  struct hna_element *e, *buff;
  int i, num_elements;

  if ((new_hna == NULL) || (new_hna_len == 0)) {
    orig_node->hna_buff = NULL;
    orig_node->hna_buff_len = 0;
  }
  else {
    orig_node->hna_buff = new unsigned char[new_hna_len];
    orig_node->hna_buff_len = new_hna_len;
    memcpy(orig_node->hna_buff, new_hna, new_hna_len);

    /* add new routes */
    num_elements = orig_node->hna_buff_len / sizeof(struct hna_element);
    buff = (struct hna_element *)orig_node->hna_buff;

    NS_LOG_DEBUG( "HNA information received (" << num_elements << " HNA network" << (num_elements > 1 ? "s": "") << "):" );

    for (i = 0; i < num_elements; i++) {
      e = &buff[i];

      if ((e->netmask > 0 ) && (e->netmask < 33))
        NS_LOG_DEBUG( "hna: " << e->addr << "/" << e->netmask );
      else
        NS_LOG_DEBUG( "hna: " << e->addr << "/" << e->netmask << " -> ignoring (invalid netmask)");

      if ((e->netmask > 0) && (e->netmask <= 32))
        _hna_global_add(orig_node, e);
    }
  }
}


void RoutingProtocol::_hna_global_add(struct orig_node *orig_node, struct hna_element *hna_element)
{
  struct hna_global_entry *hna_global_entry;
  struct orig_node *hna_orig_ptr = NULL;
  struct orig_node *old_orig_node = NULL;
  struct hashtable_t *swaphash;

  hna_global_entry = ((struct hna_global_entry *)hash_find(m_hnaGlobalHash, hna_element));

  /* add the hna node if it does not exist */
  if (!hna_global_entry) {
    hna_global_entry = new struct hna_global_entry;

    if (!hna_global_entry)
      return;

    hna_global_entry->addr = hna_element->addr;
    hna_global_entry->netmask = hna_element->netmask;
    hna_global_entry->curr_orig_node = NULL;
    
//    INIT_LIST_HEAD_FIRST(hna_global_entry->orig_list);
    
    hash_add(m_hnaGlobalHash, hna_global_entry);

    if (m_hnaGlobalHash->elements * 4 > m_hnaGlobalHash->size) {
      swaphash = hash_resize(m_hnaGlobalHash, m_hnaGlobalHash->size * 2);

      if (swaphash == NULL)
        NS_LOG_DEBUG( "Couldn't resize global hna hash table \n");
      else
        m_hnaGlobalHash = swaphash;
    }
  }

  /* the given orig_node already is the current orig node for this HNA */
  if (hna_global_entry->curr_orig_node == orig_node)
    return;

  for ( unsigned int x = 0; x < hna_global_entry->originList.size(); ++x ) {
    hna_orig_ptr = hna_global_entry->originList[x];

    if (hna_orig_ptr == orig_node)
      break;

    hna_orig_ptr = NULL;
  }

  /* append the given orig node to the list */
  if (!hna_orig_ptr)
    hna_global_entry->originList.push_back(orig_node);

  /* our TQ value towards the HNA is orig_nodebetter */
  if ((!hna_global_entry->curr_orig_node) ||
      (orig_node->router->tq_avg > hna_global_entry->curr_orig_node->router->tq_avg)) {

    old_orig_node = hna_global_entry->curr_orig_node;
    hna_global_entry->curr_orig_node = orig_node;

    /**
     * if we change the orig node towards the HNA we may still route via the same next hop
     * which does not require any routing table changes
     */
    if ((old_orig_node) &&
        (hna_global_entry->curr_orig_node->router->addr == old_orig_node->router->addr))
      return;

/*
 *
 * TODO
    add_del_route( hna_element->addr, hna_element->netmask, orig_node->router->addr,
                   orig_node->router->if_incoming->addr.sin_addr.s_addr,
                   orig_node->router->if_incoming->if_index,
                   orig_node->router->if_incoming->dev,
                   BATMAN_RT_TABLE_NETWORKS, ROUTE_TYPE_UNICAST, ROUTE_ADD );
 */
  }

  /* delete previous route */
  if (old_orig_node) {
/*
 *
 * TODO
    add_del_route( hna_element->addr, hna_element->netmask, old_orig_node->router->addr,
                   old_orig_node->router->if_incoming->addr.sin_addr.s_addr,
                   old_orig_node->router->if_incoming->if_index,
                   old_orig_node->router->if_incoming->dev,
                   BATMAN_RT_TABLE_NETWORKS, ROUTE_TYPE_UNICAST, ROUTE_DEL );
 */
  }
}


void RoutingProtocol::HNAGlobalDel(struct orig_node *orig_node)
{
  struct hna_element *e, *buff;
  int i, num_elements;

  if ( !((orig_node->hna_buff == NULL) || (orig_node->hna_buff_len == 0)) ) {
    /* delete routes */
    num_elements = orig_node->hna_buff_len / sizeof(struct hna_element);
    buff = (struct hna_element *)orig_node->hna_buff;

    for (i = 0; i < num_elements; i++) {
      e = &buff[i];

    /* not found / deleted, need to add this new route */
    if ((e->netmask > 0) && (e->netmask <= 32))
      _hna_global_del(orig_node, e);
    }

    delete orig_node->hna_buff;
    orig_node->hna_buff = NULL;
    orig_node->hna_buff_len = 0;
  }
}


void RoutingProtocol::_hna_global_del(struct orig_node *orig_node, struct hna_element *hna_element)
{
  struct hna_global_entry *hna_global_entry;
  struct orig_node *hna_orig_ptr = NULL;

  hna_global_entry = ((struct hna_global_entry *)hash_find(m_hnaGlobalHash, hna_element));

  if (hna_global_entry) {

    hna_global_entry->curr_orig_node = NULL;

    for ( unsigned int x = 0; x < hna_global_entry->originList.size(); ++x ) {
      hna_orig_ptr = hna_global_entry->originList[x];
      /* delete old entry in orig list */
      if (hna_orig_ptr == orig_node) {
        hna_global_entry->originList.erase( hna_global_entry->originList.begin() + x );
        // TODO: Do we need to do this?
//        delete hna_orig_ptr;

        // we just removed X but loop will increment, so back up one
        --x;
        continue;
      }

      /* find best alternative route */
      if ( (!hna_global_entry->curr_orig_node) ||
           (hna_orig_ptr->router->tq_avg > hna_global_entry->curr_orig_node->router->tq_avg))
        hna_global_entry->curr_orig_node = hna_orig_ptr;
    }

    /* set new route if available */
    if (hna_global_entry->curr_orig_node) {
      /**
      * if we delete one orig node towards the HNA but we switch to an alternative
      * which is reachable via the same next hop no routing table changes are necessary
      */
      if (hna_global_entry->curr_orig_node->router->addr == orig_node->router->addr)
        return;
/*
 * TODO: Fix this
      add_del_route(hna_element->addr, hna_element->netmask, hna_global_entry->curr_orig_node->router->addr,
                    hna_global_entry->curr_orig_node->router->if_incoming->addr.sin_addr.s_addr,
                    hna_global_entry->curr_orig_node->router->if_incoming->if_index,
                    hna_global_entry->curr_orig_node->router->if_incoming->dev,
                    BATMAN_RT_TABLE_NETWORKS, ROUTE_TYPE_UNICAST, ROUTE_ADD);
      }

      add_del_route(hna_element->addr, hna_element->netmask, orig_node->router->addr,
                    orig_node->router->if_incoming->addr.sin_addr.s_addr,
                    orig_node->router->if_incoming->if_index,
                    orig_node->router->if_incoming->dev,
                    BATMAN_RT_TABLE_NETWORKS, ROUTE_TYPE_UNICAST, ROUTE_DEL);
 */

      /* if no alternative route is available remove the HNA entry completely */
      if (!hna_global_entry->curr_orig_node) {
        hash_remove(m_hnaGlobalHash, hna_element);
        NS_LOG_DEBUG( "HNA Global Entry ptr: " << hna_global_entry );
      }
    }
  }
}


/**
 * hna_global_update() replaces the old add_del_hna function. This function
 * updates the new hna buffer for the supplied orig node and
 * adds/deletes/updates the announced routes.
 *
 * Instead of first deleting and then adding, we try to add new routes
 * before delting the old ones so that the kernel will not experience
 * a situation where no route is present.
 */
void RoutingProtocol::HNAGlobalUpdate(struct orig_node *orig_node, unsigned char *new_hna,
                                      int16_t new_hna_len, struct neigh_node *old_router)
{
  struct hna_element *e, *buff;
  struct hna_global_entry *hna_global_entry;
  int i, num_elements, old_hna_len;
  unsigned char *old_hna;

  /* orig node stopped announcing any networks */
  if ((orig_node->hna_buff) && ((new_hna == NULL) || (new_hna_len == 0))) {
    HNAGlobalDel(orig_node);
    return;
  }

  /* orig node started to announce networks */
  if ((!orig_node->hna_buff) && ((new_hna != NULL) || (new_hna_len != 0))) {
    HNAGlobalAdd(orig_node, new_hna, new_hna_len);
    return;
  }

  /**
   * next hop router changed - no need to change the global hna hash
   * we just have to make sure that the best orig node is still in place
   * NOTE: we may miss a changed HNA here which we will update with the next packet
   */
  if (old_router != orig_node->router) {
    num_elements = orig_node->hna_buff_len / sizeof(struct hna_element);
    buff = (struct hna_element *)orig_node->hna_buff;

    for (i = 0; i < num_elements; i++) {
      e = &buff[i];

      if ((e->netmask < 1) || (e->netmask > 32))
        continue;

      hna_global_entry = ((struct hna_global_entry *)hash_find(m_hnaGlobalHash, e));

      if (!hna_global_entry)
        continue;

      /* if the given orig node is not in use no routes need to change */
      if (hna_global_entry->curr_orig_node != orig_node)
        continue;
/*
 * TODO: add later
      add_del_route(e->addr, e->netmask, orig_node->router->addr,
          orig_node->router->if_incoming->addr.sin_addr.s_addr,
          orig_node->router->if_incoming->if_index,
          orig_node->router->if_incoming->dev,
          BATMAN_RT_TABLE_NETWORKS, ROUTE_TYPE_UNICAST, ROUTE_ADD);

      add_del_route(e->addr, e->netmask, old_router->addr,
        old_router->if_incoming->addr.sin_addr.s_addr,
        old_router->if_incoming->if_index,
        old_router->if_incoming->dev,
        BATMAN_RT_TABLE_NETWORKS, ROUTE_TYPE_UNICAST, ROUTE_DEL);
 */
    }

    return;
  }

  /**
   * check if the buffers even changed. if its still the same, there is no need to
   * update the routes. if the router changed, then we have to update all the routes
   * NOTE: no NULL pointer checking here because memcmp() just returns if n == 0
   */
  if ((orig_node->hna_buff_len == new_hna_len) && (memcmp(orig_node->hna_buff, new_hna, new_hna_len) == 0))
    return; /* nothing to do */

  /* changed HNA */
  old_hna = orig_node->hna_buff;
  old_hna_len = orig_node->hna_buff_len;

  orig_node->hna_buff = new unsigned char[new_hna_len];
  orig_node->hna_buff_len = new_hna_len;
  memcpy(orig_node->hna_buff, new_hna, new_hna_len);

  /* add new routes and keep old routes */
  num_elements = orig_node->hna_buff_len / sizeof(struct hna_element);
  buff = (struct hna_element *)orig_node->hna_buff;

  for (i = 0; i < num_elements; i++) {
    e = &buff[i];

    /**
     * if the router is the same, and the announcement was already in the old
     * buffer, we can keep the route.
     */
    if (HNABuffDelete((struct hna_element *)old_hna, &old_hna_len, e) == 0) {
      /* not found / deleted, need to add this new route */
      if ((e->netmask > 0) && (e->netmask <= 32))
        _hna_global_add(orig_node, e);
    }
  }

  /* old routes which are not to be kept are deleted now. */
  num_elements = old_hna_len / sizeof(struct hna_element);
  buff = (struct hna_element *)old_hna;

  for (i = 0; i < num_elements; i++) {
    e = &buff[i];

    if ((e->netmask > 0) && (e->netmask <= 32))
      _hna_global_del(orig_node, e);
  }

  /* dispose old hna buffer now. */
  if (old_hna != NULL)
    delete old_hna;
}


/* hna_buff_delete searches in buf if element e is found.
 *
 * if found, delete it from the buf and return 1.
 * if not found, return 0.
 */
int RoutingProtocol::HNABuffDelete(struct hna_element *buf, int *buf_len, struct hna_element *e)
{
  int i;
  int num_elements;

  if (buf == NULL)
    return 0;

  /* align to multiple of sizeof(struct hna_element) */
  num_elements = *buf_len / sizeof(struct hna_element);

  for (i = 0; i < num_elements; i++) {

    if (memcmp(&buf[i], e, sizeof(struct hna_element)) == 0) {

      /* move last element forward */
      memmove(&buf[i], &buf[num_elements - 1], sizeof(struct hna_element));
      *buf_len -= sizeof(struct hna_element);

      return 1;
    }
  }
  return 0;
}


} // namespace batmand, ns3

}

