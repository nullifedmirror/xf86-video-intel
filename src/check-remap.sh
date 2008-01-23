#!/bin/sh

LANG=C

if ! which nm 2>/dev/null >/dev/null; then
	echo "'nm' not found; skipping test"
	exit 0
fi

test -z "$srcdir" && srcdir=.
status=0

objs="dri_bufmgr.o intel_bufmgr_ttm.o"
for obj in $objs; do
	obj=.libs/${obj}

	test -f $obj || continue
	echo Checking $obj

	syms=`nm "$obj" | grep " T " | cut -d" " -f3 | grep -E "intel|dri"`
	bad_syms=`echo $syms | grep -v "ddx_"`

	if test "x$bad_syms" != "x"; then
	    echo "ERROR: $obj contains non-remapped symbols: $bad_syms"
	    status=1
	fi
done

exit $status
