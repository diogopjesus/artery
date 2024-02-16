add_opp_run(test-cpm CONFIG omnetpp.ini)
add_opp_test(test-cpm SUFFIX inet CONFIG inet SIMTIME_LIMIT 20s)

if(WITH_ENVMOD)
    add_opp_test(test-cpm SUFFIX envmod CONFIG envmod SIMTIME_LIMIT 20s)
endif()