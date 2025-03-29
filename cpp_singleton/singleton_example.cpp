/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 2025, Sergei Vinogradov
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <atomic>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>

class Object
{
public:
  Object(const std::string& name) : mName(name)
  {
    std::cout << "Default ctor Object " << mName << std::endl;
  }

  Object(const Object&) = delete;
  Object(Object&&) = delete;
  Object& operator=(const Object&) = delete;

  ~Object()
  {
    std::cout << "Destructor Object" << mName << std::endl;
  }

private:
  std::string mName;
};

class GlobalHandler
{
  using mutex_type = std::mutex;
  using LockGuard = std::lock_guard<mutex_type>;
  template <typename T> struct InstWithSpinLock
  {
    mutex_type Lock;
    std::unique_ptr<T> Inst;
  };

  template <typename T> struct InstWithOnceFlag
  {
    std::once_flag once_flag;
    std::unique_ptr<T> Inst;
  };

  template <typename T, typename... Types>
  T* getOrCreate(InstWithSpinLock<T>& IWL, Types&&... Args)
  {
    LockGuard Lock{IWL.Lock};
    if (!IWL.Inst)
      IWL.Inst = std::make_unique<T>(std::forward<Types>(Args)...);

    return IWL.Inst.get();
  }

  template <typename T, typename... Types>
  T* getOrCreate(InstWithOnceFlag<T>& IWF, Types&&... Args)
  {
    std::call_once(
        IWF.once_flag, [&]()
        { IWF.Inst = std::make_unique<T>(std::forward<Types>(Args)...); });

    return IWF.Inst.get();
  }

  template <typename T, typename... Types>
  T* getOrCreate(std::unique_ptr<T>& UPTR, Types&&... Args)
  {
    UPTR = std::make_unique<T>(std::forward<Types>(Args)...);
    return UPTR.get();
  }

  GlobalHandler() {}

  InstWithSpinLock<Object> a_instance;
  InstWithOnceFlag<Object> b_instance;
  std::unique_ptr<Object> c_instance;

public:
  static GlobalHandler& getInstance();

  Object& getInstanceA()
  {
    Object* a = getOrCreate(a_instance, "A");
    return *a;
  }

  Object& getInstanceB()
  {
    Object* b = getOrCreate(b_instance, "B");
    return *b;
  }

  Object& getInstanceC()
  {
    static Object* c = getOrCreate(c_instance, "C");
    return *c;
  }
};

GlobalHandler& GlobalHandler::getInstance()
{
  static std::unique_ptr<GlobalHandler> instance(new GlobalHandler());

  return *instance;
}

// this function is used to measure the execution time of a function
// and returns duration
template <typename Func>
std::chrono::high_resolution_clock::duration measureExecTime(Func&& func)
{
  auto start = std::chrono::high_resolution_clock::now();
  func();
  auto end = std::chrono::high_resolution_clock::now();
  return end - start;
}

template <typename Duration>
void printDuration(const Duration& duration, const std::string& experimentName)
{
  std::cout
      << "Exect time " << experimentName << ": "
      << std::chrono::duration_cast<std::chrono::microseconds>(duration).count()
      << "us" << std::endl;
}

Object& testA(GlobalHandler& gh)
{
  return gh.getInstanceA();
}

Object& testB(GlobalHandler& gh)
{
  return gh.getInstanceB();
}

Object& testC(GlobalHandler& gh)
{
  return gh.getInstanceC();
}

int main()
{
  constexpr size_t NUM_ITERATIONS = 10000000;
  GlobalHandler& gh = GlobalHandler::getInstance();

  //////////////////////////////////////////////
  std::cout << "testing gh.getInstanceA()" << std::endl;
  auto duration = measureExecTime(
      [&]()
      {
        for (size_t i = 0; i < NUM_ITERATIONS; ++i)
        {
          Object& a = testA(gh);
        }
      });
  printDuration(duration, "getInstanceA");

  //////////////////////////////////////////////
  std::cout << "testing gh.getInstanceB()" << std::endl;
  duration = measureExecTime(
      [&]()
      {
        for (size_t i = 0; i < NUM_ITERATIONS; ++i)
        {
          Object& b = testB(gh);
        }
      });
  printDuration(duration, "getInstanceB");

  //////////////////////////////////////////////
  std::cout << "testing gh.getInstanceC()" << std::endl;
  duration = measureExecTime(
      [&]()
      {
        for (size_t i = 0; i < NUM_ITERATIONS; ++i)
        {
          Object& c = testC(gh);
        }
      });
  printDuration(duration, "getInstanceC");

  return 0;
}
