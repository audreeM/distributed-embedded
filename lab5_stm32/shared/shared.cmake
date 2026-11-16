# Sources shared by both STM32 apps. Each app's CMakeLists.txt sets NODE_ID and includes this.
set(SHARED ${CMAKE_CURRENT_LIST_DIR})
set(COMMON ${CMAKE_CURRENT_LIST_DIR}/../../common)

target_sources(app PRIVATE
	${SHARED}/src/main.c
	${SHARED}/src/canbus.c
	${SHARED}/src/dispatch.c
	${SHARED}/src/edges.c
	${SHARED}/src/gtime.c
	${SHARED}/src/hb_tx.c
	${SHARED}/src/raft_task.c
	${SHARED}/src/relay.c
	${SHARED}/src/tp.c
	${SHARED}/src/zone.c
	${COMMON}/raft/raft.c
	${COMMON}/raft/raft_sm.c
	${COMMON}/raft/raft_client.c
	${COMMON}/timesync/timesync.c
)

target_include_directories(app PRIVATE
	include
	${SHARED}/include
	${COMMON}
	${COMMON}/raft
	${COMMON}/timesync
)

target_compile_definitions(app PRIVATE NODE_ID=${NODE_ID})

# Raft config for a checkoff run, e.g.  west build ... -- -DRAFT_LEADER=0xFF -DRAFT_SEED=2
if(DEFINED RAFT_LEADER)
	target_compile_definitions(app PRIVATE RAFT_DEFAULT_LEADER=${RAFT_LEADER})
endif()
if(DEFINED RAFT_SEED)
	target_compile_definitions(app PRIVATE RAFT_SEED=${RAFT_SEED})
endif()
if(DEFINED RAFT_LOG_SIZE)
	target_compile_definitions(app PRIVATE RAFT_LOG_SIZE=${RAFT_LOG_SIZE})
endif()
