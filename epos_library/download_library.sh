#!/usr/bin/env bash

if [ ! -f EPOS-Linux-Library-En.zip ]; then wget "http://www.maxonmotorusa.com/medias/sys_master/root/8815100330014/EPOS-Linux-Library-En.zip";fi
if [ ! -d EPOS_Linux_Library ]; then unzip "EPOS-Linux-Library-En.zip";fi
