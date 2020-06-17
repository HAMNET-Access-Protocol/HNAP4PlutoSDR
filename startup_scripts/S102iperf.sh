#!/bin/sh

[ -f /usr/bin/iperf3 ] || exit 0

start() {
	printf "Starting iperf3 server: "
  iperf3 -s &	
	echo "done"
}


stop() {
	printf "Stopping iperf3: "
	killall iperf3 
	echo "done"
}

restart() {
	stop
	start
}

# See how we were called.
case "$1" in
  start)
	start
	;;
  stop)
	stop
	;;
  restart|reload)
	restart
	;;
  *)
	echo "Usage: $0 {start|stop|reload|restart}"
	exit 1
esac

exit $?
