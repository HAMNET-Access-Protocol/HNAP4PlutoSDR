#!/bin/sh

[ -f /root/basestation ] || exit 0

# Autostart the basestation service

start() {
  	if [ `fw_printenv -n hnap_bs_autostart` = 1 ]
	then
		printf "Starting basestation application"
		/root/./basestation -t 0 -g 70 &
	fi
	echo "done"
}


stop() {
	printf "Stopping basestation: "
	killall basestation
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
