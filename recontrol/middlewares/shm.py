import multiprocessing as mp
import threading as th
from multiprocessing.managers import SharedMemoryManager

from ._middleware import Node
from .shared_memory import SharedMemoryQueue, SharedMemoryRingBuffer

SMM = {}


class ShmServer(Node):
    def __init__(self, *args, **kwargs):
        self.__post_init__()

    def __post_init__(self):
        pass

    def start(self):
        pass

    def stop(self):
        pass


class ShmClient(Node):
    def __init__(
        self,
        daemon: bool = True,
        shm_addr: str = "127.0.0.1:5555",  # NOTE: use same addr across all node processes
        get_time_budget: float = 0.2,
        *,
        freq: int = 100,
        max_buffer_size: int = 30,
        max_queue_size: int = 100,
        timeout: float = 5.0,
        verbose=True,
        **kwargs,
    ):
        self.daemon = daemon
        super().__init__()
        self.shm_addr = shm_addr
        self.get_time_budget = get_time_budget
        self.freq = freq
        self.max_buffer_size = max_buffer_size
        self.max_queue_size = max_queue_size
        self.timeout = timeout
        self.verbose = verbose
        # self.__post_init__()  # call in mp_run() instead
        self.post_init()

    def post_init(self):
        host, port = self.shm_addr.split(":")
        smm = SharedMemoryManager(address=(host, int(port)), authkey=b"abc")
        self.owns_smm = False
        try:
            smm.connect()
        except ConnectionRefusedError:
            smm.start()
            self.owns_smm = True
            # global reference to smm to avoid garbage collection shutting down smm (and pickling errors)
            SMM[self.shm_addr] = smm

        self.parent_conn, child_conn = mp.Pipe(duplex=False)  # to receive ring_buffer and request_queue from mp_run()

        self.ring_buffer = None
        self.request_queue = None
        self.pub_ready_event = mp.Event() if self.has_pub else None
        self.req_ready_event = mp.Event() if self.has_req else None
        self.exit_event = mp.Event()
        self.worker_thread = None  # created by mp_run()
        args = (child_conn, self.pub_ready_event, self.req_ready_event, self.exit_event)
        self.main_process = mp.Process(target=self.mp_run, args=args, daemon=self.daemon)  # self.run
        # NOTE: this class does not inherit from mp.Process to avoid pickling errors

    def start(self):
        # worker_thread will be started in main process
        self.main_process.start()

        # set ring_buffer and request_queue
        self.parent_conn.poll(timeout=self.timeout)
        self.ring_buffer, self.request_queue = self.parent_conn.recv()
        self.parent_conn.close()
        del self.parent_conn

        self.pub_ready_event.wait(timeout=self.timeout) if self.pub_ready_event is not None else None
        self.req_ready_event.wait(timeout=self.timeout) if self.req_ready_event is not None else None
        # worker_thread is also alive if main process is alive
        assert self.main_process.is_alive()

    def stop(self):
        self.exit_event.set()
        # worker_thread will also be joined when main process is joined
        self.main_process.join(self.timeout)

        if self.owns_smm:
            try:
                SMM[self.shm_addr].shutdown()
            except AttributeError:
                pass

    def __post_init__(self):
        self.run, self._run = self.mp_run, self.run

        host, port = self.shm_addr.split(":")
        smm = SharedMemoryManager(address=(host, int(port)), authkey=b"abc")
        smm.connect()

        if self.has_pub:
            assert self.example_data is not None
            self.ring_buffer = SharedMemoryRingBuffer.create_from_examples(
                shm_manager=smm,
                examples=self.example_data,
                get_max_k=self.max_buffer_size,
                get_time_budget=self.get_time_budget,
                put_desired_frequency=self.freq,
            )
        else:
            self.ring_buffer = None
        if self.has_req:
            assert self.example_request is not None
            self.request_queue = SharedMemoryQueue.create_from_examples(
                shm_manager=smm,
                examples=self.example_request,
                buffer_size=self.max_queue_size,
            )
        else:
            self.request_queue = None
        self.worker_thread = th.Thread(target=self.worker, daemon=self.daemon) if self.worker is not None else None

    def _start(self):
        self.worker_thread.start() if self.worker_thread is not None else None
        assert self.worker_thread.is_alive() if self.worker_thread is not None else True

    def _stop(self):
        self.exit_event.set()
        self.worker_thread.join(self.timeout) if self.worker_thread is not None else None

    def mp_run(self, *args):
        child_conn = args[0]
        self.pub_ready_event, self.req_ready_event, self.exit_event = args[1:]
        self.__post_init__()
        child_conn.send((self.ring_buffer, self.request_queue))
        child_conn.close()

        self._start()
        try:
            self._run()
        except KeyboardInterrupt:
            pass
        finally:
            self._stop()

    def __getstate__(self):
        state = self.__dict__.copy()
        state.pop("pub_ready_event", None)
        state.pop("req_ready_event", None)
        state.pop("exit_event", None)
        state.pop("worker_thread", None)
        state.pop("main_process", None)
        return state

    def __setstate__(self, state):
        self.__dict__.update(state)
        self.pub_ready_event = None
        self.req_ready_event = None
        self.exit_event = None
        self.worker_thread = None
        self.main_process = None
