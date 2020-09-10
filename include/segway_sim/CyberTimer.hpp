#ifndef _CUSTOM_TIMER_H_
#define _CUSTOM_TIMER_H_

#include <array>
#include <chrono>
#include <numeric>

template<int averageLen>
class CyberTimer
{
	static_assert(averageLen>0, "Averaging length must be strictly positive");

public:
	CyberTimer(void):
	i_(0),
	iterCount_(0),
	running_(false),
	dt_(0.0)
	{};

	~CyberTimer()
	{};

	void tic(void)
	{
		running_ = true;
		t_start_ = std::chrono::high_resolution_clock::now();
	};

	double toc(void)
	{
		if(running_)
		{
			running_ = false;
			const auto t_stop = std::chrono::high_resolution_clock::now();
			const auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(t_stop - t_start_);
			dt_ = static_cast<double>(duration.count())*1.0e-9;
			
			buffer_[i_] = dt_;
			i_++;
			if(i_>=averageLen)
				i_=0;
			iterCount_++;
		}

		return dt_;
	};

	double getAverage(void) const
	{
		if(iterCount_==0)
			return -1.0;

		if(iterCount_<averageLen)
			return std::accumulate(buffer_.begin(), buffer_.begin()+iterCount_, 0.0)/iterCount_;

		return std::accumulate(buffer_.begin(), buffer_.begin()+averageLen, 0.0)/averageLen;
	};

	double getAverageCount(void) const
	{
		if(iterCount_<averageLen)
			return iterCount_;
		else
			return averageLen;
	};

	bool isRunnning(void) const
	{
		return running_;
	};

	double getLatest(void) const
	{
		if(iterCount_==0)
			return -1.0;
		else
			return dt_;
	};

private:
	uint32_t i_;
	uint32_t iterCount_;
	bool running_;
	double dt_;
	std::chrono::time_point<std::chrono::high_resolution_clock> t_start_;
	std::array<double,averageLen> buffer_;
};


#endif