Demo of RTIC delay not returning.  Note that it's currently configured to get flashed via probe-rs.  Expect to see like

```
...
[TRACE] Pushing to channel @15033940
[INFO ] msg_receiver rx msg @15033994 [0]
[TRACE] msg_receiver await msg @15034028
[TRACE] Pushing to channel @15043047
[INFO ] msg_receiver rx msg @15043101 [0]
[TRACE] msg_receiver await msg @15043135
[TRACE] ...delay done @15043379
[TRACE] delay...
[TRACE] Pushing to channel @15051899
[INFO ] msg_receiver rx msg @15051952 [0]
[TRACE] msg_receiver await msg @15051986
[TRACE] ...delay done @15053426
[TRACE] delay...
[TRACE] Pushing to channel @15053565
[INFO ] msg_receiver rx msg @15053619 [0]
...
```

and within like 30 seconds `delay...` and `... delay done` stop happening because the intervening delay never returned.