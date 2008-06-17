#!/bin/bash
# svn-ignore - tell Subversion to ignore a file in certain operations.
# See: http://svn-ignore.notlong.com [svnbook.red-bean.com]

test -z "$1" && echo "Usage: $0 FILENAME" && exit -1

for fullname in "$@"
do
  dirname=$(dirname "$fullname")
  filename=$(basename "$fullname")
  (svn propget svn:ignore "$dirname" | egrep -v '^$';
   echo "$filename") >/tmp/svn-ignore.$$
  svn propset svn:ignore -F /tmp/svn-ignore.$$ "$dirname"
  rm /tmp/svn-ignore.$$
done
