#!/bin/bash
# 仅设置 MOX 独立 USB 扬声器，不影响 ReSpeaker 麦克风
amixer -c UACDemoV10 sset 'PCM' 100% unmute
