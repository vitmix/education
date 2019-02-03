/* Simple Observer pattern. Subject can add observers (listeners),
 * notify observers about changes in his state and delete observers.
 * Observer must have "update" function notifying him of a change of
 * the Subject.
 */
#include <iostream>
#include <functional>
#include <vector>
#include <memory>
#include <string>

template <typename T>
class Subject final {
	//description of the notify function
	using listener = std::function<void (Subject<T>*)>;
public:
	Subject() : value_{} {}
	~Subject() = default;

	Subject(const Subject &) = delete;
	Subject(Subject &&) = delete;
	Subject & operator=(const Subject &) = delete;
	Subject & operator=(Subject &&) = delete;

	//add listener to the vector of shared_ptr
	void pushValueListener(listener l) {
		valueListeners.push_back(std::make_shared<listener> (l));
	}

	//after changing internal state of Subject,
	//we should notify about that all observers
	void setValue(T val) {
		if (val != value_) {
			value_ = val;
			updateValueListener();
		}
	}

	operator T() { return value_; }

	//to demonstrate the use of weak_ptr 
	//we artificially make a "dangling" pointer
	void popBackValueListener() {
		(valueListeners.back()).reset();
	}

	//this func removes the last
	//element of the vector 
	void deleteBackValueListener() {
		valueListeners.pop_back();
	}
private:
	//this func notifies observers and
	//demonstrates the use of weak_ptr
	void updateValueListener() {

		//for all elements in vector
		for (auto &f : valueListeners) {

			//we create weak_ptr
			std::weak_ptr<listener> wptr(f);

			//and calling observer`s notify functions
			//if pointer is not dangling
			if (!wptr.expired()) { (*f) (this); }
			else std::cout << "is expired! \n";
		}
	}

	T value_;
	std::vector<std::shared_ptr<listener>> valueListeners;
};

template <typename T>
class Observer {
public:
	Observer(std::string s) : s_(s) {}
	~Observer() = default;

	void update(Subject<T> *subj) {
		std::cout << s_ << " saw Subject`s value updated to "
			<< *subj << "\n";
	}
private:
	std::string s_;
};

int main()
{
	Subject<int> subj;

	subj.setValue(19);
	std::cout << "there is no observers, value is : " << subj << "\n";

	//first notify function
	auto lambdaObserver1 = [](Subject<int> *s) {
		std::cout << "New Subject value (lambdaObserver1): " << *s << "\n";
	};

	//adding first observer
	subj.pushValueListener(lambdaObserver1);

	//second notify function
	auto lambdaObserver2 = [](Subject<int> *s) {
		std::cout << "New Subject value : (lambdaObserver2)" << *s << "\n";
	};

	//adding second observer
	subj.pushValueListener(lambdaObserver2);

	std::cout << "\n";

	//changing state of Subject
	subj.setValue(-20);

	//making dangling pointer
	subj.popBackValueListener();

	std::cout << "\n";

	//another changing to demonstrate using of weak_ptr
	subj.setValue(30);

	subj.deleteBackValueListener();

	//third observer
	Observer<int> ob("instance of Observer");
	subj.pushValueListener([&ob](Subject<int> *s) {
		//explicit calling of notify function
		ob.update(s);
	});

	std::cout << "\n";

	subj.setValue(99);

	system("pause");
	return 0;
}
