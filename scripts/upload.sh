#!/bin/bash

# go to https://cs452.student.cs.uwaterloo.ca/
# hover over user-name in top-right corner; then click 'Profile'
# generate new and/or obtain current API token and store in $HOME/.cs452_token

[ -f $HOME/.cs452_token ] || {
    echo "API token in $HOME/.cs452_token not found"
    exit 1
}

[ $# -lt 2 ] && {
    echo "usage: $0 <image> <mac>"
    exit 1
}

[ -f $1 ] || {
    echo "image file $1 not found"
    exit 1
}

MAC=$(echo $2|tr A-Z a-z|tr ':' '-')

case "$MAC" in
    a|tracka) MAC=d8-3a-dd-1b-36-9e;;
    d8-3a-dd-1b-36-9e) ;;
    d8-3a-dd-1b-37-13) ;;
    d8-3a-dd-1b-37-d9) ;;
    d8-3a-dd-1b-39-65) ;;
    d8-3a-dd-1b-36-7d) ;;
    b|trackb) MAC=d8-3a-dd-1b-38-6c;;
    d8-3a-dd-1b-38-6c) ;;
    *) echo "MAC address $MAC not in permitted list"; exit 1;;
esac

echo uploading $1 to $MAC

curl -F "upload_file=@$1" -F "api_token=$(cat $HOME/.cs452_token)" \
 -L https://cs452.student.cs.uwaterloo.ca/api/raspberrypi/$MAC/kernel
echo