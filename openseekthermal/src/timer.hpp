// Copyright (c) 2023 Stefan Fabian. All rights reserved.
// Licensed under the MIT license.

#ifndef HECTOR_TIMEIT_TIMER_HPP
#define HECTOR_TIMEIT_TIMER_HPP

#include <cassert>
#include <chrono>
#include <memory>
#include <vector>

#ifdef __unix__

#include <time.h>
#include <unistd.h>

#endif

namespace hector_timeit
{

/*!
 * Timer class that can be used for simple profiling.
 * The runtime of a single method can be measured using the static time method.
 * To measure multiple runs use a Timer instance and pass true to the reset method between runs.
 */
class Timer
{
public:
  enum TimeUnit { Default = 0, Seconds = 1, Milliseconds = 2, Microseconds = 3, Nanoseconds = 4 };

  static inline bool getCpuTime( long &val )
  {
    // This could also maybe made a parameter
#ifdef _POSIX_THREAD_CPUTIME
    struct timespec spec;
    if ( clock_gettime( CLOCK_THREAD_CPUTIME_ID, &spec ) == -1 ) {
      return false;
    }
    val = spec.tv_sec * 1000L * 1000L * 1000L + spec.tv_nsec;
#elif defined( _POSIX_CPUTIME )
    struct timespec spec;
    if ( clock_gettime( CLOCK_PROCESS_CPUTIME_ID, &spec ) == -1 ) {
      return false;
    }
    val = spec.tv_sec * 1000L * 1000L * 1000L + spec.tv_nsec;
#else
    val = std::clock() * 1000L * 1000L * 1000L / CLOCKS_PER_SEC;
#endif
    return true;
  }

  /*!
   * Constructs a new Timer instance.
   * @param name: The name of the timer. Used for printing in the toString method and stream operator.
   * @param print_time_unit The time unit used for printing. If Default the time unit is automatically chosen.
   * @param autostart If true, the timer starts immediately after construction. If false, it has to be manually started
   *  using the start() method.
   * @param print_on_destruct If true, prints when the Time object is destructed.
   */
  explicit Timer( std::string name, TimeUnit print_time_unit = Default, bool autostart = true,
                  bool print_on_destruct = false );

  ~Timer();

  template<typename Fn>
  static auto time( Fn &&fn, const std::string &name, TimeUnit print_time_unit )
  {
    Timer timer( name, print_time_unit, true, true );
    return fn();
  }

#if __cplusplus > 201703L
private:
  static std::string sourceLocToString( const std::source_location &location )
  {
    return location.file_name() + ":" + std::to_string( location.line() );
  }

public:
  // C++ version is greater than C++17 (C++20 or later)
  template<typename Fn>
  static auto time( Fn &&fn,
                    const std::string &name = sourceLocToString( std::source_location::current() ) )
#else
  // C++ version is C++17 or earlier
  template<typename Fn>
  static auto time( Fn &&fn,
#ifdef __GNUC__
                    // Add your code here for GCC
                    const std::string &name = std::string( "anonymous (" ) + __builtin_FUNCTION() +
                                              std::string( ":" ) +
                                              std::to_string( __builtin_LINE() ) + ")" )
#else
                    const std::string &name = "anonymous timer" )
#endif
#endif
  {
    return time<Fn>( std::forward<Fn>( fn ), name, TimeUnit::Default );
  }

  const std::string &name() const { return name_; }

  /*!
   * Starts the timer if it isn't already running.
   */
  inline void start()
  {
    if ( running_ )
      return;
    running_ = true;
    /*
     * To get a more accurate measurement, the time it takes to measure the time is subtracted by using the following method:
     * We assume that each measurement takes roughly the same time
     * Time measurement is split into _I which is time call initialization (time until the time is measured) and _R
     *  which is time call return time (time until the measured time is returned)
     * WALL_START_A    XI XR
     * WALL_START_B    XI XR
     * CPU_START_A     YI YR
     * CPU_START_B     YI YR
     * RUN_TIME        R
     * CPU_END_B       YI YR
     * CPU_END_A       YI YR
     * WALL_END_B      XI XR
     * WALL_END_A      XI XR
     *
     * CPU A duration: YR + YI + YR + R + YI + YR + YI
     * CPU B duration:           YR + R + YI
     * Diff CPU = CPU A - CPU B = YR + YI + YR + YI = 2 * (YR + YI)
     * CPU Time = CPU B - 1/2 * Diff CPU = R
     *
     * Wall time A duration: XR + XI + XR + YI + YR + YI + YR + R + YI + YR + YI + YR + XI + XR + XI
     * Wall time B duration:           XR + YI + YR + YI + YR + R + YI + YR + YI + YR + XI
     * Diff Wall time = Wall time A - Wall time B = XR + XI + XR + XI = 2 * (XR + XI)
     * Wall time = Wall time B - 1/2 * Diff Wall time - 2 * Diff CPU = 1.5 * Wall time B - 0.5 * Wall time A - 2 * Diff CPU = R
     */
    start_a_ = std::chrono::high_resolution_clock::now();
    start_b_ = std::chrono::high_resolution_clock::now();
    cpu_time_valid_a_ = getCpuTime( cpu_start_a_ );
    cpu_time_valid_b_ = getCpuTime( cpu_start_b_ );
  }

  /*!
   * Stops the timer if it isn't already stopped. Timing can be resumed using the start method.
   */
  inline void stop()
  {
    if ( !running_ )
      return;
    // See start method for a documentation of the algorithm used to get precise time measurements
    long time_a = 0;
    long time_b = 0;
    if ( cpu_time_valid_a_ ) {
      if ( cpu_time_valid_b_ ) {
        cpu_time_valid_b_ = getCpuTime( time_b );
      }
      cpu_time_valid_a_ = getCpuTime( time_a );
    }
    auto time_point_b = std::chrono::high_resolution_clock::now();
    auto time_point_a = std::chrono::high_resolution_clock::now();
    long cpu_diff = 0;
    if ( cpu_time_valid_a_ ) {
      long elapsed;
      if ( cpu_time_valid_b_ ) {
        cpu_diff = ( time_a - cpu_start_a_ ) - ( time_b - cpu_start_b_ );
        elapsed = ( time_b - cpu_start_b_ ) - cpu_diff / 2;
      } else {
        elapsed = time_a - cpu_start_a_;
      }
      assert( elapsed >= 0 );
      elapsed_cpu_time_ += elapsed;
    }
    long wall_time_b = internalGetDuration( start_b_, time_point_b );
    long wall_time_a = internalGetDuration( start_a_, time_point_a );
    long elapsed = wall_time_b + wall_time_b / 2 - wall_time_a / 2 - 2 * cpu_diff;

    assert( elapsed >= 0 );
    elapsed_time_ += elapsed;
    running_ = false;
  }

  /*!
   * Resets the timer. If new_run is false all runs are cleared as well.
   * If you want to time multiple runs pass true.
   * @param new_run Whether or not you want to time a new run. If false, everything is reset including the runs.
   */
  void reset( bool new_run = false );

  /*!
   * Returns the elapsed time since the timer or run was started excluding the time where it was
   * paused using the stop method. Does not use any logic to account for the timing function call.
   * @return The elapsed time in nanoseconds.
   */
  inline long getElapsedTime() const
  {
    long result = elapsed_time_;
    if ( running_ )
      result += internalGetDuration( start_b_, std::chrono::high_resolution_clock::now() );
    return result;
  }

  /*!
   * Returns the elapsed cpu or thread time (depending on what is available) since the timer or run was started
   *  excluding the time where it was paused using the stop method.
   * Does not use any logic to account for the timing function call.
   * @return The elapsed cpu or thread time in nanoseconds.
   */
  inline long getElapsedCpuTime() const
  {
    if ( !cpu_time_valid_a_ )
      return -1;
    long result = elapsed_cpu_time_;
    if ( running_ ) {
      long time;
      if ( !getCpuTime( time ) )
        return -1;
      result += time - cpu_start_a_;
    }
    return result;
  }

  std::vector<long> getRunTimes() const;

  std::vector<long> getCpuRunTimes() const;

  std::string toString() const;

protected:
  static std::string internalPrint( const std::string &name, const std::vector<long> &run_times,
                                    const std::vector<long> &cpu_run_times,
                                    TimeUnit print_time_unit );

  static inline long internalGetDuration( const std::chrono::high_resolution_clock::time_point &start,
                                          const std::chrono::high_resolution_clock::time_point &end )
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>( end - start ).count();
  }

  std::vector<long> run_times_;
  std::vector<long> cpu_run_times_;
  std::string name_;
  TimeUnit print_time_unit_;
  std::chrono::high_resolution_clock::time_point start_a_;
  std::chrono::high_resolution_clock::time_point start_b_;
  long elapsed_time_ = 0;
  long elapsed_cpu_time_ = 0;
  long cpu_start_a_ = 0;
  long cpu_start_b_ = 0;
  bool running_ = false;
  bool cpu_time_valid_a_ = true;
  bool cpu_time_valid_b_ = true;
  bool print_on_destruct_ = false;
};

/*!
 * @brief Helper class to measure individual runs on a timer.
 * Starts the timer on construction and stops and resets the next run timer on destruction.
 */
struct TimeBlock {
  explicit TimeBlock( Timer &timer ) : timer_( timer ) { timer_.start(); }

  ~TimeBlock() { end(); }

  void end()
  {
    timer_.stop();
    if ( ended_ )
      return;
    ended_ = true;
    timer_.reset( true );
  }

  Timer &timer_;
  bool ended_ = false;
};

} // namespace hector_timeit

std::ostream &operator<<( std::ostream &stream, const hector_timeit::Timer &timer );

// IMPL
#include <cmath>
#include <iostream>
#include <sstream>

namespace hector_timeit
{

inline Timer::Timer( std::string name, TimeUnit print_time_unit, bool autostart,
                     bool print_on_destruct )
    : name_( std::move( name ) ), print_time_unit_( print_time_unit ),
      print_on_destruct_( print_on_destruct )
{
  if ( autostart )
    start();
}

inline Timer::~Timer()
{
  if ( print_on_destruct_ )
    std::cout << *this << std::endl << std::flush;
}

inline void Timer::reset( bool new_run )
{
  stop();
  if ( new_run ) {
    if ( elapsed_time_ > 0 ) {
      run_times_.push_back( elapsed_time_ );
      cpu_run_times_.push_back( cpu_time_valid_a_ ? elapsed_cpu_time_ : -1 );
    }
  } else {
    run_times_.clear();
    cpu_run_times_.clear();
  }
  elapsed_time_ = 0;
  elapsed_cpu_time_ = 0;
  cpu_time_valid_a_ = true;
  cpu_time_valid_b_ = true;
}

inline std::vector<long> Timer::getRunTimes() const
{
  std::vector<long> result = run_times_;
  long elapsed_time = getElapsedTime();
  if ( elapsed_time != 0 ) {
    result.push_back( elapsed_time );
  }
  return result;
}

inline std::vector<long> Timer::getCpuRunTimes() const
{
  std::vector<long> result = cpu_run_times_;
  long elapsed_cpu_time = getElapsedCpuTime();
  if ( elapsed_cpu_time > 0 ) {
    result.push_back( elapsed_cpu_time );
  }
  return result;
}

inline std::string Timer::toString() const
{
  return internalPrint( name_, getRunTimes(), getCpuRunTimes(), print_time_unit_ );
}

inline void printPaddedString( std::ostringstream &stream, const std::string &text, size_t pad = 0 )
{
  size_t i = 0;
  for ( ; pad != 0 && i < ( pad - text.length() ) / 2; ++i ) stream << " ";
  stream << text;
  for ( i += text.length(); i < pad; ++i ) stream << " ";
}

template<typename T>
void printTimeString( std::ostringstream &outstream, T time, Timer::TimeUnit print_time_unit,
                      int pad = 0 )
{
  std::ostringstream stream;
  stream.precision( 3 );
  stream.setf( std::ios::fixed, std::ios::floatfield );
  switch ( print_time_unit ) {
  case Timer::Seconds:
    stream << time / 1E9 << "s";
    break;
  case Timer::Milliseconds:
    stream << time / 1E6 << "ms";
    break;
  case Timer::Microseconds:
    stream << time / 1000.0 << "us";
    break;
  case Timer::Nanoseconds:
    stream << time << "ns";
    break;
  case Timer::Default:
  default:
    if ( time < 5000 ) {
      stream << time << "ns";
    } else if ( time < 5E6 ) {
      stream << time / 1E3 << "us";
    } else if ( time < 5E9 ) {
      stream << time / 1E6 << "ms";
    } else {
      stream << time / 1E9 << "s";
    }
    break;
  }
  printPaddedString( outstream, stream.str(), pad );
}

inline double square( double x ) { return x * x; }

inline void printStats( std::ostringstream &stream, const std::vector<long> &run_times,
                        Timer::TimeUnit print_time_unit )
{
  long max = 0;
  long min = INT64_MAX;
  long long sum = 0;
  size_t count = 0;
  for ( long time : run_times ) {
    if ( time == -1 )
      continue;
    sum += time;
    ++count;
    if ( time > max )
      max = time;
    if ( time < min )
      min = time;
  }
  double mean = (double)sum / count;
  double var = 0;
  for ( long time : run_times ) {
    if ( time == -1 )
      continue;
    var += square( time - mean );
  }
  var /= ( count - 1 );
  if ( count == 0 ) {
    stream << "None of the runs had valid times!";
    return;
  }
  // Average
  std::ostringstream avg_stream;
  printTimeString( avg_stream, mean, print_time_unit, 0 );
  avg_stream << " +- ";
  printTimeString( avg_stream, sqrt( var ), print_time_unit, 0 );
  printPaddedString( stream, avg_stream.str(), 40 );
  // Longest
  printTimeString( stream, max, print_time_unit, 16 );
  // Shortest
  printTimeString( stream, min, print_time_unit, 16 );
  // Sum
  printTimeString( stream, sum, print_time_unit, 16 );
  if ( count != run_times.size() ) {
    stream << std::endl
           << "Warning: Only " << count << " of " << run_times.size() << " had valid times!";
  }
}

inline std::string Timer::internalPrint( const std::string &name, const std::vector<long> &run_times,
                                         const std::vector<long> &cpu_run_times,
                                         TimeUnit print_time_unit )
{
  std::ostringstream stringstream;
  stringstream << "[Timer: " << name << "] " << run_times.size() << " run(s) took: ";
  if ( run_times.empty() ) {
    stringstream << "no time at all.";
  } else if ( run_times.size() == 1 ) {
    printTimeString( stringstream, run_times[0], print_time_unit, 0 );
    if ( cpu_run_times[0] != -1 ) {
#ifdef _POSIX_THREAD_CPUTIME
      printPaddedString( stringstream, " (Thread: ", 0 );
#else
      printPaddedString( stringstream, " (CPU: ", 0 );
#endif
      printTimeString( stringstream, cpu_run_times[0], print_time_unit, 0 );
      stringstream << ")";
    }
    stringstream << ".";
  } else {
    stringstream << std::endl;
    printPaddedString( stringstream, "Type", 8 );
    printPaddedString( stringstream, "Mean (+/- stddev)", 40 );
    printPaddedString( stringstream, "Longest", 16 );
    printPaddedString( stringstream, "Shortest", 16 );
    printPaddedString( stringstream, "Sum", 16 );
    stringstream << std::endl;
    printPaddedString( stringstream, "Real", 8 );
    printStats( stringstream, run_times, print_time_unit );
    stringstream << std::endl;
#ifdef _POSIX_THREAD_CPUTIME
    printPaddedString( stringstream, "Thread", 8 );
#else
    printPaddedString( stringstream, "CPU", 8 );
#endif
    printStats( stringstream, cpu_run_times, print_time_unit );
  }
  return stringstream.str();
}
} // namespace hector_timeit

inline std::ostream &operator<<( std::ostream &stream, const hector_timeit::Timer &timer )
{
  return stream << timer.toString();
}

#endif // HECTOR_TIMEIT_TIMER_HPP
