/* 046267 Computer Architecture - Spring 2020 - HW #4 */

#include "core_api.h"
#include "sim_api.h"

#include <stdio.h>

using namespace std;

enum STATE {WAITING, READY, HALT};

struct ThreadContext{
    unsigned thread_id;
    int ticks; // timer until data from mem will be ready
    STATE state; // 0 = halt, 1 = running
    unsigned instruction_mem_addr;
    tcontext *register_file;

    ThreadContext() = default;
    ~ThreadContext() {
        delete register_file;
    };

    void _initialize_register_file() {
        for (int reg_idx = 0; reg_idx < REGS_COUNT; ++reg_idx) {
            register_file->reg[reg_idx] = 0;
        }
    }

    void initialize(const unsigned tid) {
        thread_id = tid;
        instruction_mem_addr = 0;
        ticks = 0;
        state = READY;
        register_file = new tcontext;
        _initialize_register_file();
    }

    void updateTicks() {
        if(this->ticks > 0) this->ticks--;
        if((ticks == 0) && (state == WAITING)) this->state = READY;
    }

    void executeThread() {
        Instruction instruction_context;
        SIM_MemInstRead(instruction_mem_addr,&instruction_context, thread_id);
        int src2_value = (instruction_context.isSrc2Imm ? instruction_context.src2_index_imm : register_file->reg[instruction_context.src2_index_imm]);

        switch (instruction_context.opcode) {
            case CMD_NOP:
                break;
            case CMD_ADDI:
            case CMD_ADD:
                register_file->reg[instruction_context.dst_index] = register_file->reg[instruction_context.src1_index] + src2_value;
                break;
            case CMD_SUBI:
            case CMD_SUB:
                register_file->reg[instruction_context.dst_index] = register_file->reg[instruction_context.src1_index] - src2_value;
                break;
            case CMD_LOAD:
                SIM_MemDataRead((register_file->reg[instruction_context.src1_index] + src2_value), &register_file->reg[instruction_context.dst_index]);
                ticks = SIM_GetLoadLat();
                break;
            case CMD_STORE:
                SIM_MemDataWrite(register_file->reg[instruction_context.dst_index] + src2_value, register_file->reg[instruction_context.src1_index]);
                ticks = SIM_GetStoreLat();
                break;
            case CMD_HALT:
                state = HALT;
                break;
        }
        instruction_mem_addr++;
        // we need one cycle for execution
        ticks++;
        if(state == READY) state = WAITING;
    }

};

struct CPU_BlockedMT {
    ThreadContext *threads_array;
    unsigned current_worker_id;
    int thread_count;
    int switch_penalty;
    int program_run_cycles = 0;

    CPU_BlockedMT() : current_worker_id(0), program_run_cycles(0) {
        this->thread_count = SIM_GetThreadsNum();
        this->switch_penalty = SIM_GetSwitchCycles();
        this->threads_array = new ThreadContext[thread_count];
        for (int thread_id = 0; thread_id < thread_count; ++thread_id) {
            threads_array[thread_id].initialize(thread_id);
        }
    };

    ~CPU_BlockedMT() {
        delete[] threads_array;
    };

    bool _all_threads_halted() const {
        int number_of_finished_threads = 0;

        for (int i = 0; i < thread_count; ++i) {
            if(threads_array[i].state == HALT) {
                number_of_finished_threads++;
            }
        }

        return (number_of_finished_threads == thread_count);
    }

    void _update_tick() const {
        for (int current_thread = 0; current_thread < thread_count; ++current_thread) {
            threads_array[current_thread].updateTicks();
        }
    }

    unsigned _calc_next_thread_id() const {
        // initialize next thread id to return
        unsigned next_thread_id = current_worker_id;

        // find the next ready thread who's NOT the current thread
        do {
            next_thread_id = (next_thread_id + 1) % thread_count;
        } while (threads_array[next_thread_id].state != READY && next_thread_id != current_worker_id);

        return next_thread_id;
    }

    unsigned _get_next_thread() const {

        unsigned next_thread_id;
        // for the first clock cycle we initialize thread id to 0 as requested in the assignment
        if(program_run_cycles == 1) {
            next_thread_id = 0;
        }

        // current worker can continue to run (no event happened, no need to switch)
        else if(threads_array[current_worker_id].state == READY) {
            next_thread_id = current_worker_id;
        }

        // not the creation of the world and current worker went to sleep
        else {
            next_thread_id = _calc_next_thread_id();
        }

        return next_thread_id;
    }

    bool _need_to_switch(const unsigned next_thread_id) const {
        return ((threads_array[next_thread_id].state == READY) && (next_thread_id != current_worker_id));
    }

    void _context_switch(const unsigned next_worker_id) {
        if(threads_array[current_worker_id].state == READY) return; // we can cont running current worker

        program_run_cycles += switch_penalty;
        for (int cycle = 0; cycle < switch_penalty; ++cycle) {
            for (int current_thread_id = 0; current_thread_id < thread_count; ++current_thread_id) {
                threads_array[current_thread_id].updateTicks();
            }
        }
        current_worker_id = next_worker_id;
    }

    void runThreads() {
        while (!_all_threads_halted()) {

            // update total cycle count
            program_run_cycles++;

            // update tick for all threads
            _update_tick();

            // get next thread id for execution
            unsigned next_thread_id = _get_next_thread();

            // if we need to perform context switch, i.e., the next thread is ready
            if(_need_to_switch(next_thread_id)) {
                _context_switch(next_thread_id);
            }

            if(threads_array[current_worker_id].state == READY) {
                threads_array[current_worker_id].executeThread();
            }
        }
    }

    double instructionsExecuted() const {
        double instruction_counter = 0;
        for (int thread_id = 0; thread_id < thread_count; ++thread_id) {
            instruction_counter += threads_array[thread_id].instruction_mem_addr;
        }
        return instruction_counter;
    }
};

struct CPU_FineGrainedMT {
    ThreadContext *threads_array;
    unsigned current_worker_id;
    int thread_count;
    int program_run_cycles;

    CPU_FineGrainedMT() : current_worker_id(0), program_run_cycles(0) {
        this->thread_count = SIM_GetThreadsNum();
        this->threads_array = new ThreadContext[thread_count];
        for (int thread_id = 0; thread_id < thread_count; ++thread_id) {
            threads_array[thread_id].initialize(thread_id);
        }
    };

    ~CPU_FineGrainedMT() {
        delete[] threads_array;
    };

    bool _all_threads_halted() const {
        int number_of_finished_threads = 0;

        for (int i = 0; i < thread_count; ++i) {
            if(threads_array[i].state == HALT) {
                number_of_finished_threads++;
            }
        }

        return (number_of_finished_threads == thread_count);
    }

    void _update_tick() const {
        for (int current_thread = 0; current_thread < thread_count; ++current_thread) {
            threads_array[current_thread].updateTicks();
        }
    }

    unsigned _calc_next_thread_id() const {
        // initialize next thread id to return
        unsigned next_thread_id = current_worker_id;


        // find the next ready thread who's NOT the current thread
        do {
            next_thread_id = (next_thread_id + 1) % thread_count;
        } while (threads_array[next_thread_id].state != READY && next_thread_id != current_worker_id);

        return next_thread_id;
    }

    unsigned _get_next_thread() const {

        unsigned next_thread_id;
        // for the first clock cycle we initialize thread id to 0 as requested in the assignment
        if(program_run_cycles == 1) {
            next_thread_id = 0;
        }

        // not the creation of the world
        else {
            next_thread_id = _calc_next_thread_id();
        }

        return next_thread_id;
    }

    void runThreads() {
        while (!_all_threads_halted()) {

            // update total cycle count
            program_run_cycles++;

            // update tick for all threads
            _update_tick();

            // get next thread id for execution
            unsigned next_thread_id = _get_next_thread();

            // if we need to perform context switch, i.e., the next thread is ready
            if(next_thread_id != current_worker_id) {
                current_worker_id = next_thread_id;
            }

            if(threads_array[current_worker_id].state == READY) {
                threads_array[current_worker_id].executeThread();
            }

            // else we're in idle state
        }
    }

    double instructionsExecuted() const {
        double instruction_counter = 0;
        for (int thread_id = 0; thread_id < thread_count; ++thread_id) {
            instruction_counter += threads_array[thread_id].instruction_mem_addr;
        }
        return instruction_counter;
    }
};

CPU_BlockedMT* cpuBlockedMt;
CPU_FineGrainedMT* cpuFineGrainedMt;

void CORE_BlockedMT() {
    cpuBlockedMt = new CPU_BlockedMT();
    cpuBlockedMt->runThreads();
}

void CORE_FinegrainedMT() {
    cpuFineGrainedMt = new CPU_FineGrainedMT;
    cpuFineGrainedMt->runThreads();
}

double CORE_BlockedMT_CPI(){
    double cpi = ((double)(cpuBlockedMt->program_run_cycles)/(cpuBlockedMt->instructionsExecuted()));
    delete cpuBlockedMt;
    return cpi;
}

double CORE_FinegrainedMT_CPI(){
    double cpi = ((double)(cpuFineGrainedMt->program_run_cycles)/(cpuFineGrainedMt->instructionsExecuted()));
    delete cpuFineGrainedMt;
    return cpi;
}

void CORE_BlockedMT_CTX(tcontext* context, int threadid) {
    for (int register_idx = 0; register_idx < REGS_COUNT; ++register_idx) {
        context[threadid].reg[register_idx] = cpuBlockedMt->threads_array[threadid].register_file->reg[register_idx];
    }
}

void CORE_FinegrainedMT_CTX(tcontext* context, int threadid) {
    for (int register_idx = 0; register_idx < REGS_COUNT; ++register_idx) {
        context[threadid].reg[register_idx] = cpuBlockedMt->threads_array[threadid].register_file->reg[register_idx];
    }
}
