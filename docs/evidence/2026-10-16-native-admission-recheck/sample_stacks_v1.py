"""GDB-only bounded stack snapshots of the unchanged full workload."""
import gdb
import json
import os
from pathlib import Path
import signal
import threading
import time
for command in ['set pagination off', 'set width 0', 'set confirm off', 'set debuginfod enabled off', 'set disable-randomization off', 'set print thread-events off', 'handle SIGINT stop print nopass']:
    gdb.execute(command)
state={'pid':0,'exit_code':None}
def created(event):
    state['pid']=event.inferior_thread.inferior.pid
def exited(event):
    state['exit_code']=getattr(event,'exit_code',None)
gdb.events.new_thread.connect(created)
gdb.events.exited.connect(exited)
samples=[]
started=time.monotonic()
for index in range(60):
    def interrupt():
        pid=state['pid']
        if pid:
            try:os.kill(pid,signal.SIGINT)
            except ProcessLookupError:pass
    timer=threading.Timer(15,interrupt)
    timer.daemon=True
    timer.start()
    try:
        gdb.execute('run' if index==0 else 'continue')
    except KeyboardInterrupt:
        pass
    finally:
        timer.cancel()
    if not gdb.selected_inferior().pid:
        break
    samples.append({'index':index,'elapsed_seconds':time.monotonic()-started})
    print('CSGRS_STACK_SAMPLE '+json.dumps(samples[-1]),flush=True)
    gdb.execute('thread apply all bt 48')
else:
    print('CSGRS_STACK_DIAGNOSTIC_CUTOFF',flush=True)
    gdb.execute('kill')
    state['exit_code']=124
record={'samples':samples,'inferior_exit_code':state['exit_code'],'seconds':time.monotonic()-started,'interval_seconds':15,'maximum_snapshots':60,'scope':'Debugger snapshots only; pauses and debugger overhead invalidate timing comparisons. Original full fixtures and assertions remain unchanged.'}
Path(os.environ['CSGRS_STACK_RESULT']).write_text(json.dumps(record,indent=2)+'\n')
print('CSGRS_STACK_RESULT '+json.dumps(record),flush=True)
gdb.execute('quit '+str(state['exit_code'] if state['exit_code'] is not None else 125))
