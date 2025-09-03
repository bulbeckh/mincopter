

// TODO This is quick but bad hack to fix persistent compiler issues - find longer term solution

extern "C" {

	void __cxa_pure_virtual() {
		for(;;);
	}
}

