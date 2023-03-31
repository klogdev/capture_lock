#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>

std::mutex mtx;
std::condition_variable cv;
std::queue<int> data_queue;

void producer() {
    for (int i = 0; i < 10; ++i) {
        std::unique_lock<std::mutex> lock(mtx);
        data_queue.push(i);
        lock.unlock();
        cv.notify_one();
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // simulate work
    }
}

void consumer() {
    while (true) {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, []{return !data_queue.empty();}); // wait for data
        int data = data_queue.front();
        data_queue.pop();
        lock.unlock();
        std::cout << "Consumed data: " << data << std::endl;
    }
}

int main() {
    std::thread producer_thread(producer);
    std::thread consumer_thread(consumer);
    producer_thread.join();
    consumer_thread.join();
    return 0;
}
