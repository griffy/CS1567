Since we're judged on code style as well, I think we should all follow a style guideline. I propose the following (feel free to add to it or change it):

## Classes

	// class names look like this
	class ClassName

	// private instance methods look like this
	_instanceMethod()

	// public instance methods look like this
	instanceMethod()

	// private instance variables look like this
	_instanceVariable

	// public instance variables look like this
	instanceVariable

## Example class definition

	class ExampleClass {
	// public first
	public:
		ExampleClass();
		void instanceMethod();
	// then private
	private:
		// instance vars first
		int _instanceVar;

		// then instance methods
		void _instanceMethod();
	};

## Variables

	// look like this
	Type variableOfType

## Indentation
4 spaces (or tab, I suppose, since whitespace doesn't matter here like it would in Python)

	// if statements
	if (condition) {
		
	}
	else {
		
	}

	// switch statements
	switch (var) {
	case 1:
		// ...
	}

	// loops
	for (int i = 0; i < 10; i++) {
		
	}

	while (true) {
		
	}