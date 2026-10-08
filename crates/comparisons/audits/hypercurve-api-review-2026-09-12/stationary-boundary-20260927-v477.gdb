set pagination off
set confirm off
set debuginfod enabled off
set print frame-arguments none
set print entry-values no
python
import gdb
class BoundaryBreakpoint(gdb.Breakpoint):
    hits=0
    def stop(self):
        self.hits+=1
        if self.hits<=16:
            print('BLOCKER_CALL '+str(self.hits))
            frame=gdb.newest_frame()
            for index in range(26):
                if frame is None:break
                print(str(index)+' '+str(frame.name()))
                frame=frame.older()
        return False
BoundaryBreakpoint('hypercurve::error::ExactCurveError::blocked')
end
run
quit
