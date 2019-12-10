ref=`git symbolic-ref HEAD 2>/dev/null | cut -b 12-`
if [ -z $ref ]; then
    ref=`git describe --tags`
fi
echo $ref-`git rev-parse --short=7 HEAD`
touch version.cpp
