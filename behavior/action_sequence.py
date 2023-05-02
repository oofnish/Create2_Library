import enum
import multiprocessing
import select
import threading

import createlib as cl


class UPDATE_RESULT(enum.IntEnum):
    OK = 0x01  # Normal, all is okay update result
    DONE = 0x02  # Done, individual action has completed its task.
    TERMINATE = 0x03 # Something went wrong, terminate everything
    BREAK = 0x04  # Action has done work, and no further actions should respond to the most recent event
    PASS = 0x09   # Pass, use for when an action is not active or is otherwise skipped


class EVENT_TYPE(enum.IntEnum):
    MESSAGE = 0x01  # Arbitrary  message event, with text data, can pass between actions
    TIMER = 0x02  # Timer event, generally a heartbeat, frequently sent message
    PAUSE = 0x03  # Pause event, Pause activity with the expectation of later resume
    RESUME = 0x04  # Resume event, Resume paused activity
    BEGIN = 0x05  # Begin event, Sent at the start of an action set
    FINISH = 0x06  # Finish event, Terminate action
    COLLIDE = 0x07  # Collide event, Bumper collision
    FRONT = 0x08  # Front event, Light bumper notification of impending collision


class EventBeginAction:
    type = EVENT_TYPE.BEGIN

    def __init__(self):
        pass


class EventMessage:
    type = EVENT_TYPE.MESSAGE

    def __init__(self, msg):
        self.data = msg


class EventTimer:
    type = EVENT_TYPE.TIMER

    def __init__(self, delta_t):
        self.data = delta_t


class EventPause:
    type = EVENT_TYPE.PAUSE

    def __init__(self):
        pass


class EventResume:
    type = EVENT_TYPE.RESUME

    def __init__(self):
        pass


class EventFinish:
    type = EVENT_TYPE.FINISH

    def __init__(self):
        pass


class EventCollide:
    type = EVENT_TYPE.COLLIDE

    def __init__(self):
        pass


class EventFront:
    type = EVENT_TYPE.FRONT

    def __init__(self):
        pass


class EventQueue:
    def __init__(self):
        self.r, self.w = multiprocessing.Pipe(False)

    def put(self, evt):
        self.w.send(evt)

    def get(self):
        return self.r.recv()

    def fileno(self):
        return self.r.fileno()


class ActionSequence(threading.Thread):
    """
    ActionSequence manages a sequence of activities to be undertaken by the robot.  This is a serial list of actions,
    such as move forward, turn, move back, beep, wait, etc.
    Additionally, one or more EventQueues can be added, which can provide asynchronous signalling to the actions.  By
    default, a single "heartbeat" event queue is created, which triggers regularly, sending a time delta to the active
    action.  Other options for event queues include Cancel and Pause events.
    Upon startup, The ActionSequence runs the first configured action, then waits for events, using a select.select()
    call on all queues.
    """

    def __init__(self, polling_rate):
        self.action_sets = []
        self.current_action = 0
        self.event_queues = []
        self.exception_queues = []

        self.heartbeat = EventQueue()
        self.next_action = EventQueue()
        self.event_queues.append(self.heartbeat)
        self.event_queues.append(self.next_action)

        self.polling_rate = polling_rate

        super().__init__(daemon=True)
        self.start()

    def append(self, action_set):
        self.action_sets.append(action_set)
        if len(self.action_sets) == self.current_action + 1:
            self.next_action.put(EventBeginAction())

    def register_event(self, evt):
        self.event_queues.append(evt)

    def register_exception(self, exevt):
        self.exception_queues.append(exevt)

    def run(self):
        print("Starting ActionSequence")
        hb_timer = cl.RepeatTimer(self.polling_rate / 1000, self.heartbeat.put, EventTimer(self.polling_rate),
                                  autostart=True)
        while True:

            evt_list, _, ex_list = select.select(self.event_queues, [], self.exception_queues)
            for e in ex_list:
                for a in self.action_sets[self.current_action]:
                    a.cancel(e.get())
                # self.action_sets[self.current_action].cancel(e.get())
                # we only need to respond to the first exception?

                hb_timer.stop()
                return
            # Flag for whether we want to start the next action after all the events are processed. This is set to
            # True if any action update returns Done.  We use a flag to ensure all current events are forwarded to
            # the current action no matter what order they appear.
            start_next_action = False
            for e in evt_list:
                evt = e.get()
                if evt.type == EVENT_TYPE.BEGIN:
                    for a in self.action_sets[self.current_action]:
                        a.begin()
                    # self.actions[self.current_action].begin()
                    continue

                if self.current_action > len(self.action_sets) - 1:
                    # no current action, so just consume incoming events
                    continue
                # pass the event details to the current action set for processing
                for a in self.action_sets[self.current_action]:
                    result = a.update(evt)
                    # result = self.action_sets[self.current_action].update(evt)
                    match result:
                        case UPDATE_RESULT.BREAK:
                            # Skip remaining processing of action set, this is a "consumed event" case
                            break
                        case UPDATE_RESULT.OK | UPDATE_RESULT.PASS:
                            # Nothing to do here, the update was all good
                            continue
                        case UPDATE_RESULT.DONE:
                            # Action set is done, get ready to move to the next one
                            start_next_action = True
                        case UPDATE_RESULT.TERMINATE:
                            hb_timer.stop()
                            # We possibly want to post an exception here instead, for better cleanup
                            return
                        case _:
                            print("UNDEFINED RESULT!")
            # check our flag; if any of the events resulted in the action reaching completion, we want to go to the next
            if start_next_action:
                self.current_action += 1
                if len(self.action_sets) > self.current_action:
                    print("Starting next action")
                    self.next_action.put(EventBeginAction())
                else:
                    print("Action sequence completed")
