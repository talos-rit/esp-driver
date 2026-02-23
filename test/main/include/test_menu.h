#ifndef _TEST_MENU_H_
#define _TEST_MENU_H_

#include <stddef.h>
typedef void (*test_group_runner_fn)(void);

typedef struct {
    const char          *name;
    test_group_runner_fn runner;
} test_group_t;

void test_menu_run(const test_group_t *groups, size_t count);

#endif /* _TEST_MENU_H_ */
