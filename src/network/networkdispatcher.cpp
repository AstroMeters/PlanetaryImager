/*
 * GuLinux Planetary Imager - https://github.com/GuLinux/PlanetaryImager
 * Copyright (C) 2016  Marco Gulino <marco@gulinux.net>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "networkdispatcher.h"
#include "networkreceiver.h"
#include "network/networkpacket.h"
#include <QtNetwork/QTcpSocket>
#include <QHash>
#include <QHash>
#include "commons/utils.h"
#include <QCoreApplication>
#include "Qt/qt_strings_helper.h"
#include "commons/definitions.h"
#if DEVELOPER_MODE == 1
#include "commons/loghandler.h"
#endif

using namespace std;

DPTR_IMPL(NetworkDispatcher) {
  QSet<NetworkReceiver *> receivers;
  QTcpSocket *socket = nullptr;
  void readyRead();
  uint64_t written;
  uint64_t sent;
  void debugPacket(const NetworkPacketPtr &packet, const QString &prefix);
};



NetworkDispatcher::NetworkDispatcher(QObject* parent) : QObject{parent}, dptr()
{
  static bool metatypes_registered = false;
  if(!metatypes_registered) {
    metatypes_registered = true;
    qRegisterMetaType<NetworkPacketPtr>("NetworkPacketPtr");
  }
}

NetworkDispatcher::~NetworkDispatcher()
{
}

void NetworkDispatcher::attach(NetworkReceiver* receiver)
{
  d->receivers.insert(receiver);
}

void NetworkDispatcher::detach(NetworkReceiver* receiver)
{
  d->receivers.remove(receiver);
}

void NetworkDispatcher::setSocket(QTcpSocket* socket)
{
  //delete d->socket;
  if(d->socket)
    d->socket->disconnect(this, 0);
  d->socket = socket;
  if(! socket)
    return;
  d->written = 0;
  d->sent = 0;
  connect(socket, &QTcpSocket::bytesWritten, this, [=](qint64 written){
    d->sent += written;
    emit bytes(d->written, d->sent);
  });
  connect(socket, &QTcpSocket::readyRead, this, bind(&Private::readyRead, d.get()));
}

void NetworkDispatcher::send(const NetworkPacketPtr &packet) {
  if(! is_connected() || ! packet)
    return;
  //qDebug() << packet->name();
  d->debugPacket(packet, ">>>");
  auto written = packet->sendTo(d->socket);
  d->written += written;
}

void NetworkDispatcher::queue_send(const NetworkPacketPtr& packet)
{
  if(packet)
    QMetaObject::invokeMethod(this, "send", Q_ARG(NetworkPacketPtr, packet) );
}


void NetworkDispatcher::Private::readyRead()
{
  QList<NetworkPacketPtr> packets;
  while(socket->bytesAvailable() > 0) {
    //qDebug() << socket->bytesAvailable();
    auto packet = make_shared<NetworkPacket>();
    packet->receiveFrom(socket);
    packets.push_back(packet);
    //qDebug() << packet->name();
  }
  for(auto packet: packets) {
  debugPacket(packet, "<<<");

    for(auto receiver: receivers)
      receiver->handle(packet);
  }
}

bool NetworkDispatcher::is_connected() const
{
  return d->socket && d->socket->isValid() && d->socket->isOpen();
}

void NetworkDispatcher::Private::debugPacket(const NetworkPacketPtr& packet, const QString& prefix)
{
#if DEBUG_NETWORK_PACKETS == 1
    QString payload;
    if(packet->payloadVariant().isValid()) {
        QDebug(&payload) << packet->payloadVariant();
    } else {
        QDebug(&payload) << packet->payload().left(180);
    }
    QString s = "%1 %2|%3\n" % prefix % packet->name() % payload;
    QMessageLogContext context;
    context.category = qPrintable("NETWORK_DEBUG");
    LogHandler::log(QtMsgType::QtDebugMsg, context, s);
#endif
}

