#!/bin/bash

ssh-keygen -t rsa -b 4096 -f ~/.ssh/id_rsa -N ""
ssh-copy-id pi@10.1.31.230
