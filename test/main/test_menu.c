/* ═══════════════════════════════════════════════════════════════
 *  test_menu.c  —  Simple serial test menu
 * ═══════════════════════════════════════════════════════════════ */
#include <stdio.h>
#include <string.h>
#include "unity.h"
#include "unity_fixture.h"
#include "test_menu.h"


/* ───────────────────────────────────────────────────────────────
 *  FRAMEWORK: Internal state
 * ─────────────────────────────────────────────────────────────── */
static const test_group_t *s_groups      = NULL;
static size_t              s_group_count = 0;

static int s_last_total   = 0;
static int s_last_failed  = 0;
static int s_last_ignored = 0;


/* ───────────────────────────────────────────────────────────────
 *  FRAMEWORK: UART input — echoes chars so cursor feels alive
 * ─────────────────────────────────────────────────────────────── */
static int read_line(char *buf, size_t max)
{
    size_t i = 0;

    /* Print prompt once before user starts typing */
    printf("> ");
    fflush(stdout);

    while (i < max - 1) {
        int c = getchar();
        if (c == EOF) continue;

        if (c == '\n' || c == '\r') {
            break;
        }

        /* Handle backspace — erase char from terminal too */
        if (c == 0x08 || c == 0x7F) {
            if (i > 0) {
                i--;
                printf("\b \b");
                fflush(stdout);
            }
            continue;
        }

        buf[i++] = (char)c;
        putchar(c);
        fflush(stdout);
    }

    buf[i] = '\0';
    putchar('\n');
    return (int)i;
}


/* ───────────────────────────────────────────────────────────────
 *  FRAMEWORK: Run helpers
 * ─────────────────────────────────────────────────────────────── */
static void capture_results(void)
{
    s_last_total   = Unity.NumberOfTests;
    s_last_failed  = Unity.TestFailures;
    s_last_ignored = Unity.TestIgnores;
}

static void run_group(const test_group_t *g)
{
    printf("\n  Running: %s\n", g->name);
    printf("  --------\n");
    UNITY_BEGIN();
    g->runner();
    UNITY_END();
    capture_results();
}

static void run_all(void)
{
    printf("\n  Running: all groups\n");
    printf("  --------\n");
    UNITY_BEGIN();
    for (size_t i = 0; i < s_group_count; i++) {
        printf("\n  [ %s ]\n", s_groups[i].name);
        s_groups[i].runner();
    }
    UNITY_END();
    capture_results();
}


/* ───────────────────────────────────────────────────────────────
 *  FRAMEWORK: Menu rendering
 * ─────────────────────────────────────────────────────────────── */
static void print_menu(void)
{
    printf("\n");
    printf("  --------------------------\n");
    printf("   Test Menu\n");
    printf("  --------------------------\n");
    printf("   a  run all\n");
    printf("   r  last results\n");
    printf("   q  quit\n");
    printf("  --------------------------\n");

    for (size_t i = 0; i < s_group_count; i++) {
        printf("   %-2zu %s\n", i, s_groups[i].name);
    }

    printf("  --------------------------\n");
}

static void print_last_results(void)
{
    printf("\n");
    if (s_last_total == 0) {
        printf("  No tests run yet.\n\n");
        return;
    }

    int passed = s_last_total - s_last_failed - s_last_ignored;

    printf("  --------------------------\n");
    printf("   Last results\n");
    printf("  --------------------------\n");
    printf("   Total   %d\n", s_last_total);
    printf("   Passed  %d\n", passed);
    printf("   Failed  %d\n", s_last_failed);
    printf("   Ignored %d\n", s_last_ignored);
    printf("  --------------------------\n\n");
}


/* ───────────────────────────────────────────────────────────────
 *  USER: Call this from app_main — pass your groups array
 * ─────────────────────────────────────────────────────────────── */
void test_menu_run(const test_group_t *groups, size_t count)
{
    s_groups      = groups;
    s_group_count = count;

    char buf[16];

    print_menu();

    while (1) {
        read_line(buf, sizeof(buf));

        if (strcmp(buf, "a") == 0) {
            run_all();
            print_menu();

        } else if (strcmp(buf, "r") == 0) {
            print_last_results();

        } else if (strcmp(buf, "q") == 0) {
            printf("\n  bye.\n\n");
            break;

        } else {
            char *end;
            long idx = strtol(buf, &end, 10);
            if (end != buf && idx >= 0 && idx < (long)s_group_count) {
                run_group(&s_groups[idx]);
                print_menu();
            } else if (strlen(buf) > 0) {
                printf("  unknown command.\n");
            }
        }
    }
}
