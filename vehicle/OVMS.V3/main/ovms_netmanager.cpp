/*
;    Project:       Open Vehicle Monitor System
;    Date:          14th March 2017
;
;    Changes:
;    1.0  Initial release
;
;    (C) 2011       Michael Stegen / Stegen Electronics
;    (C) 2011-2017  Mark Webb-Johnson
;    (C) 2011        Sonny Chen @ EPRO/DX
;
; Permission is hereby granted, free of charge, to any person obtaining a copy
; of this software and associated documentation files (the "Software"), to deal
; in the Software without restriction, including without limitation the rights
; to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
; copies of the Software, and to permit persons to whom the Software is
; furnished to do so, subject to the following conditions:
;
; The above copyright notice and this permission notice shall be included in
; all copies or substantial portions of the Software.
;
; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
; IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
; AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
; OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
; THE SOFTWARE.
*/

#include "ovms_log.h"
static const char *TAG = "netmanager";

#include <string.h>
#include <stdio.h>
#include <lwip/netif.h>
#include "metrics_standard.h"
#include "ovms_peripherals.h"
#include "ovms_netmanager.h"
#include "ovms_command.h"

OvmsNetManager MyNetManager __attribute__ ((init_priority (8999)));

void network_status(int verbosity, OvmsWriter* writer, OvmsCommand* cmd, int argc, const char* const* argv)
  {
  struct netif *ni = netif_list;
  while (ni)
    {
    writer->printf("Interface#%d: %c%c\n",ni->num,ni->name[0],ni->name[1]);
    writer->printf("  IPv4: " IPSTR "/" IPSTR " gateway " IPSTR "\n",
      IP2STR(&ni->ip_addr.u_addr.ip4), IP2STR(&ni->netmask.u_addr.ip4), IP2STR(&ni->gw.u_addr.ip4));
    ni = ni->next;
    writer->puts("");
    }

  writer->printf("DNS:");
  int dnsservers = 0;
  for (int k=0;k<DNS_MAX_SERVERS;k++)
    {
    ip_addr_t srv = dns_getserver(k);
    if (! ip_addr_isany(&srv))
      {
      dnsservers++;
      if (srv.type == IPADDR_TYPE_V4)
        writer->printf(" " IPSTR, IP2STR(&srv.u_addr.ip4));
      else if (srv.type == IPADDR_TYPE_V6)
        writer->printf(" " IPSTR, IP2STR(&srv.u_addr.ip6));
      }
    }
  if (dnsservers == 0)
    writer->puts(" None");
  else
    writer->puts("");
  }

OvmsNetManager::OvmsNetManager()
  {
  ESP_LOGI(TAG, "Initialising NETMANAGER (8999)");
  m_connected_wifi = false;
  m_connected_modem = false;
  m_connected_any = false;

#ifdef CONFIG_OVMS_SC_GPL_MONGOOSE
  m_mongoose_task = 0;
  m_mongoose_running = false;
#endif //#ifdef CONFIG_OVMS_SC_GPL_MONGOOSE

  // Register our commands
  OvmsCommand* cmd_network = MyCommandApp.RegisterCommand("network","NETWORK framework",network_status, "", 0, 1);
  cmd_network->RegisterCommand("status","Show network status",network_status, "", 0, 0);

  // Register our events
  #undef bind  // Kludgy, but works
  using std::placeholders::_1;
  using std::placeholders::_2;
  MyEvents.RegisterEvent(TAG,"system.wifi.sta.gotip", std::bind(&OvmsNetManager::WifiUp, this, _1, _2));
  MyEvents.RegisterEvent(TAG,"system.wifi.ap.start", std::bind(&OvmsNetManager::WifiUp, this, _1, _2));
  MyEvents.RegisterEvent(TAG,"system.wifi.sta.stop", std::bind(&OvmsNetManager::WifiDown, this, _1, _2));
  MyEvents.RegisterEvent(TAG,"system.wifi.ap.stop", std::bind(&OvmsNetManager::WifiDown, this, _1, _2));
  MyEvents.RegisterEvent(TAG,"system.wifi.sta.disconnected", std::bind(&OvmsNetManager::WifiDown, this, _1, _2));
  MyEvents.RegisterEvent(TAG,"system.wifi.down", std::bind(&OvmsNetManager::WifiDown, this, _1, _2));
  MyEvents.RegisterEvent(TAG,"system.modem.gotip", std::bind(&OvmsNetManager::ModemUp, this, _1, _2));
  MyEvents.RegisterEvent(TAG,"system.modem.stop", std::bind(&OvmsNetManager::ModemDown, this, _1, _2));
  MyEvents.RegisterEvent(TAG,"system.modem.down", std::bind(&OvmsNetManager::ModemDown, this, _1, _2));
  }

OvmsNetManager::~OvmsNetManager()
  {
  }

void OvmsNetManager::WifiUp(std::string event, void* data)
  {
  m_connected_wifi = true;
  m_connected_any = m_connected_wifi || m_connected_modem;
  StandardMetrics.ms_m_net_type->SetValue("wifi");
#ifdef CONFIG_OVMS_COMP_WIFI
  StandardMetrics.ms_m_net_provider->SetValue(MyPeripherals->m_esp32wifi->GetSSID());
#endif // #ifdef CONFIG_OVMS_COMP_WIFI
  MyEvents.SignalEvent("network.wifi.up",NULL);
  MyEvents.SignalEvent("network.up",NULL);
#ifdef CONFIG_OVMS_SC_GPL_MONGOOSE
  StartMongooseTask();
#endif //#ifdef CONFIG_OVMS_SC_GPL_MONGOOSE
  }

void OvmsNetManager::WifiDown(std::string event, void* data)
  {
  if (m_connected_wifi)
    {
    m_connected_wifi = false;
    m_connected_any = m_connected_wifi || m_connected_modem;
    MyEvents.SignalEvent("network.wifi.down",NULL);
    if (m_connected_any)
      MyEvents.SignalEvent("network.reconfigured",NULL);
    else
      {
      StandardMetrics.ms_m_net_type->SetValue("none");
      StandardMetrics.ms_m_net_provider->SetValue("");
      MyEvents.SignalEvent("network.down",NULL);
#ifdef CONFIG_OVMS_SC_GPL_MONGOOSE
      StopMongooseTask();
#endif //#ifdef CONFIG_OVMS_SC_GPL_MONGOOSE
      }
    }
  }

void OvmsNetManager::ModemUp(std::string event, void* data)
  {
  m_connected_modem = true;
  m_connected_any = m_connected_wifi || m_connected_modem;
  StandardMetrics.ms_m_net_type->SetValue("modem");
  MyEvents.SignalEvent("network.modem.up",NULL);
  MyEvents.SignalEvent("network.up",NULL);
#ifdef CONFIG_OVMS_SC_GPL_MONGOOSE
  StartMongooseTask();
#endif //#ifdef CONFIG_OVMS_SC_GPL_MONGOOSE
  }

void OvmsNetManager::ModemDown(std::string event, void* data)
  {
  if (m_connected_modem)
    {
    m_connected_modem = false;
    m_connected_any = m_connected_wifi || m_connected_modem;
    MyEvents.SignalEvent("network.modem.down",NULL);
    if (m_connected_any)
      MyEvents.SignalEvent("network.reconfigured",NULL);
    else
      {
      StandardMetrics.ms_m_net_type->SetValue("none");
      StandardMetrics.ms_m_net_provider->SetValue("");
      MyEvents.SignalEvent("network.down",NULL);
#ifdef CONFIG_OVMS_SC_GPL_MONGOOSE
      StopMongooseTask();
#endif //#ifdef CONFIG_OVMS_SC_GPL_MONGOOSE
      }
    }
  }

#ifdef CONFIG_OVMS_SC_GPL_MONGOOSE

static void MongooseRawTask(void *pvParameters)
  {
  OvmsNetManager* me = (OvmsNetManager*)pvParameters;
  me->MongooseTask();
  }

void OvmsNetManager::MongooseTask()
  {
  // Initialise the mongoose manager
  mg_mgr_init(&m_mongoose_mgr, NULL);
  MyEvents.SignalEvent("network.mgr.init",NULL);

  // Main event loop
  while (m_mongoose_running)
    {
    mg_mgr_poll(&m_mongoose_mgr, 1000);
    }

  // Shutdown cleanly
  MyEvents.SignalEvent("network.mgr.stop",NULL);
  mg_mgr_free(&m_mongoose_mgr);
  vTaskDelete(NULL);
  }

struct mg_mgr* OvmsNetManager::GetMongooseMgr()
  {
  return &m_mongoose_mgr;
  }

bool OvmsNetManager::MongooseRunning()
  {
  return m_mongoose_running;
  }

void OvmsNetManager::StartMongooseTask()
  {
  if (!m_mongoose_running)
    {
    m_mongoose_running = true;
    xTaskCreatePinnedToCore(MongooseRawTask, "NetManTask", 7*1024, (void*)this, 5, &m_mongoose_task, 1);
    }
  }

void OvmsNetManager::StopMongooseTask()
  {
  m_mongoose_running = false;
  }

#endif //#ifdef CONFIG_OVMS_SC_GPL_MONGOOSE
