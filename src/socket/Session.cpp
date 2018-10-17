#include "Session.h"

Session::Session(tcp::socket socket, TcpServer* server, LinkUpNode* node)
	: socket_(std::move(socket))
{
	server_ = server;
	node_ = node;
	address_ = socket_.remote_endpoint().address();
	port_ = socket_.remote_endpoint().port();

	dataIn_ = (uint8_t*)calloc(max_length, sizeof(uint8_t));
	dataOut1_ = (uint8_t*)calloc(max_length, sizeof(uint8_t));
	dataOut2_ = (uint8_t*)calloc(max_length, sizeof(uint8_t));

	node->reset();
	length1_ = node_->getRaw(dataOut1_, max_length);
}

void Session::read()
{
	auto self(shared_from_this());
	socket_.async_read_some(boost::asio::buffer(dataIn_, max_length),
		[this, self](boost::system::error_code ec, std::size_t length)
	{
		if (ec == boost::system::errc::errc_t::success)
		{
#ifdef LINKUP_DEBUG_DETAIL
			if (length > 0) {
				for (int j = 0; j < length; j++) {
					logFile.setf(ios::hex, ios::basefield);
					logFile << (int)dataIn_[j] << " ";
					logFile.unsetf(ios::hex);
				}
			}
			if (length > 0)
				std::cout << length << std::endl;
#endif //LINKUP_DEBUG_DETAIL

			node_->progress(dataIn_, length, LinkUpProgressType::Input);
			if (length == 0)
			{
				boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
			}
			read();
		}
		else
		{
			server_->removeSession(this);
			return;
		}
	});
}

Session::~Session()
{
	free(dataIn_);
	free(dataOut1_);
	free(dataOut2_);

#ifdef LINKUP_DEBUG_DETAIL
	logFile.close();
#endif //LINKUP_DEBUG_DETAIL
}

void Session::start()
{
	this->write();
	this->read();

#ifdef LINKUP_DEBUG_DETAIL
	logFile.open("dump.txt");
#endif //LINKUP_DEBUG_DETAIL
}

void Session::write()
{
	auto self(shared_from_this());
	boost::system::error_code error;

	try
	{
		mtx.lock();

		length2_ = length1_;

		uint8_t* pTemp = dataOut2_;
		dataOut2_ = (uint8_t*)dataOut1_;
		dataOut1_ = pTemp;

		if (length2_ == 0)
		{
			boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
		}

		boost::asio::async_write(socket_, boost::asio::buffer(dataOut2_, length2_),
			[this, self](boost::system::error_code ec, std::size_t length)
		{
			if (ec == boost::system::errc::errc_t::success)
			{
				write();
			}
			else
			{
				server_->removeSession(this);
				return;
			}
		});

		length1_ = node_->getRaw(dataOut1_, max_length);

		mtx.unlock();
	}
	catch (std::exception& e)
	{
		std::cerr << "Exception in thread: " << e.what() << "\n";
	}
}

