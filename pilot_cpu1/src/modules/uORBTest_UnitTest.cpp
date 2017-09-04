/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "uORBTest_UnitTest.hpp"
#include "uORBCommon.hpp"
#include <stdio.h>

uORBTest::UnitTest &uORBTest::UnitTest::instance()
{
	static uORBTest::UnitTest t;
	return t;
}

int uORBTest::UnitTest::pubsublatency_main(void)
{
	/* poll on test topic and output latency */
	float latency_integral = 0.0f;

	/* wakeup source(s) */
	struct pollfd fds[3];

	int test_multi_sub = orb_subscribe_multi(ORB_ID(orb_test), 0);
	int test_multi_sub_medium = orb_subscribe_multi(ORB_ID(orb_test_medium), 0);
	int test_multi_sub_large = orb_subscribe_multi(ORB_ID(orb_test_large), 0);

	struct orb_test_large t;

	/* clear all ready flags */
	orb_copy(ORB_ID(orb_test), test_multi_sub, &t);
	orb_copy(ORB_ID(orb_test_medium), test_multi_sub_medium, &t);
	orb_copy(ORB_ID(orb_test_large), test_multi_sub_large, &t);

	fds[0].fd = test_multi_sub;
	fds[0].events = POLLIN;
	fds[1].fd = test_multi_sub_medium;
	fds[1].events = POLLIN;
	fds[2].fd = test_multi_sub_large;
	fds[2].events = POLLIN;

	const unsigned maxruns = 1000;
	unsigned timingsgroup = 0;

	unsigned *timings = new unsigned[maxruns];

	for (unsigned i = 0; i < maxruns; i++) {
		/* wait for up to 500ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 500);

		if (fds[0].revents & POLLIN) {
			orb_copy(ORB_ID(orb_test), test_multi_sub, &t);
			timingsgroup = 0;

		} else if (fds[1].revents & POLLIN) {
			orb_copy(ORB_ID(orb_test_medium), test_multi_sub_medium, &t);
			timingsgroup = 1;

		} else if (fds[2].revents & POLLIN) {
			orb_copy(ORB_ID(orb_test_large), test_multi_sub_large, &t);
			timingsgroup = 2;
		}

		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		hrt_abstime elt = hrt_elapsed_time(&t.time);
		latency_integral += elt;
		timings[i] = elt;
	}

	orb_unsubscribe(test_multi_sub);
	orb_unsubscribe(test_multi_sub_medium);
	orb_unsubscribe(test_multi_sub_large);

	if (pubsubtest_print) {
		char fname[32];
		sprintf(fname, PX4_ROOTFSDIR"/fs/microsd/timings%u.txt", timingsgroup);
		FILE *f = fopen(fname, "w");

		if (f == NULL) {
			pilot_warn("Error opening file!\n");
			return uORB::ERROR;
		}

		for (unsigned i = 0; i < maxruns; i++) {
			fprintf(f, "%u\n", timings[i]);
		}

		fclose(f);
	}

	delete[] timings;

	pilot_warn("mean: %8.4f", static_cast<double>(latency_integral / maxruns));

	pubsubtest_passed = true;

	if (static_cast<float>(latency_integral / maxruns) > 30.0f) {
		pubsubtest_res = uORB::ERROR;

	} else {
		pubsubtest_res = 0;
	}

	return pubsubtest_res;
}

int uORBTest::UnitTest::test()
{
	int ret = test_single();

	if (ret != OK) {
		return ret;
	}

	ret = test_multi();

	if (ret != OK) {
		return ret;
	}

	ret = test_multi_reversed();

	if (ret != OK) {
		return ret;
	}

	return OK;
}


int uORBTest::UnitTest::info()
{
	return OK;
}

int uORBTest::UnitTest::test_single()
{
	pilot_info("try single-topic support");

	struct orb_test t, u;
	int sfd;
	orb_advert_t ptopic;
	bool updated;

	t.val = 0;
	ptopic = orb_advertise(ORB_ID(orb_test), &t);

	if (ptopic == NULL) {
		return pilot_err("advertise failed: %d", errno);
	}

	pilot_info("publish handle 0x%08x", ptopic);
	sfd = orb_subscribe(ORB_ID(orb_test));

	if (sfd < 0) {
		return pilot_err("subscribe failed: %d", errno);
	}

	pilot_info("subscribe fd %d", sfd);
	u.val = 1;

	if (0 != orb_copy(ORB_ID(orb_test), sfd, &u)) {
		return pilot_err("copy(1) failed: %d", errno);
	}

	if (u.val != t.val) {
		return pilot_err("copy(1) mismatch: %d expected %d", u.val, t.val);
	}

	if (0 != orb_check(sfd, &updated)) {
		return pilot_err("check(1) failed");
	}

	if (updated) {
		return pilot_err("spurious updated flag");
	}

	t.val = 2;
	pilot_info("try publish");

	if (0 != orb_publish(ORB_ID(orb_test), ptopic, &t)) {
		return pilot_err("publish failed");
	}

	if (0 != orb_check(sfd, &updated)) {
		return pilot_err("check(2) failed");
	}

	if (!updated) {
		return pilot_err("missing updated flag");
	}

	if (0 != orb_copy(ORB_ID(orb_test), sfd, &u)) {
		return pilot_err("copy(2) failed: %d", errno);
	}

	if (u.val != t.val) {
		return pilot_err("copy(2) mismatch: %d expected %d", u.val, t.val);
	}

	orb_unsubscribe(sfd);

	return pilot_info("PASS single-topic test");
}

int uORBTest::UnitTest::test_multi()
{
	/* this routine tests the multi-topic support */
	pilot_info("try multi-topic support");

	struct orb_test t, u;
	t.val = 0;
	int instance0;
	orb_advert_t pfd0 = orb_advertise_multi(ORB_ID(orb_multitest), &t, &instance0, ORB_PRIO_MAX);

	pilot_info("advertised");

	int instance1;
	orb_advert_t pfd1 = orb_advertise_multi(ORB_ID(orb_multitest), &t, &instance1, ORB_PRIO_MIN);

	if (instance0 != 0) {
		return pilot_err("mult. id0: %d", instance0);
	}

	if (instance1 != 1) {
		return pilot_err("mult. id1: %d", instance1);
	}

	t.val = 103;

	if (0 != orb_publish(ORB_ID(orb_multitest), pfd0, &t)) {
		return pilot_err("mult. pub0 fail");
	}

	pilot_info("published");

	t.val = 203;

	if (0 != orb_publish(ORB_ID(orb_multitest), pfd1, &t)) {
		return pilot_err("mult. pub1 fail");
	}

	/* subscribe to both topics and ensure valid data is received */
	int sfd0 = orb_subscribe_multi(ORB_ID(orb_multitest), 0);

	if (0 != orb_copy(ORB_ID(orb_multitest), sfd0, &u)) {
		return pilot_err("sub #0 copy failed: %d", errno);
	}

	if (u.val != 103) {
		return pilot_err("sub #0 val. mismatch: %d", u.val);
	}

	int sfd1 = orb_subscribe_multi(ORB_ID(orb_multitest), 1);

	if (0 != orb_copy(ORB_ID(orb_multitest), sfd1, &u)) {
		return pilot_err("sub #1 copy failed: %d", errno);
	}

	if (u.val != 203) {
		return pilot_err("sub #1 val. mismatch: %d", u.val);
	}

	/* test priorities */
	int prio;

	if (0 != orb_priority(sfd0, &prio)) {
		return pilot_err("prio #0");
	}

	if (prio != ORB_PRIO_MAX) {
		return pilot_err("prio: %d", prio);
	}

	if (0 != orb_priority(sfd1, &prio)) {
		return pilot_err("prio #1");
	}

	if (prio != ORB_PRIO_MIN) {
		return pilot_err("prio: %d", prio);
	}

	if (0 != latency_test<struct orb_test>(ORB_ID(orb_test), false)) {
		return pilot_err("latency test failed");
	}

	return pilot_info("PASS multi-topic test");
}

int uORBTest::UnitTest::test_multi_reversed()
{
	pilot_info("try multi-topic support subscribing before publishing");

	/* For these tests 0 and 1 instances are taken from before, therefore continue with 2 and 3. */

	/* Subscribe first and advertise afterwards. */
	int sfd2 = orb_subscribe_multi(ORB_ID(orb_multitest), 2);

	if (sfd2 < 0) {
		return pilot_err("sub. id2: ret: %d", sfd2);
	}

	struct orb_test t, u;

	t.val = 0;

	int instance2;

	orb_advert_t pfd2 = orb_advertise_multi(ORB_ID(orb_multitest), &t, &instance2, ORB_PRIO_MAX);

	int instance3;

	orb_advert_t pfd3 = orb_advertise_multi(ORB_ID(orb_multitest), &t, &instance3, ORB_PRIO_MIN);

	pilot_info("advertised");

	if (instance2 != 2) {
		return pilot_err("mult. id2: %d", instance2);
	}

	if (instance3 != 3) {
		return pilot_err("mult. id3: %d", instance3);
	}

	t.val = 204;

	if (0 != orb_publish(ORB_ID(orb_multitest), pfd2, &t)) {
		return pilot_err("mult. pub0 fail");
	}


	t.val = 304;

	if (0 != orb_publish(ORB_ID(orb_multitest), pfd3, &t)) {
		return pilot_err("mult. pub1 fail");
	}

	pilot_info("published");

	if (0 != orb_copy(ORB_ID(orb_multitest), sfd2, &u)) {
		return pilot_err("sub #2 copy failed: %d", errno);
	}

	if (u.val != 204) {
		return pilot_err("sub #3 val. mismatch: %d", u.val);
	}

	int sfd3 = orb_subscribe_multi(ORB_ID(orb_multitest), 3);

	if (0 != orb_copy(ORB_ID(orb_multitest), sfd3, &u)) {
		return pilot_err("sub #3 copy failed: %d", errno);
	}

	if (u.val != 304) {
		return pilot_err("sub #3 val. mismatch: %d", u.val);
	}

	return pilot_info("PASS multi-topic reversed");
}


void uORBTest::UnitTest::pubsubtest_threadEntry(void *pvParameters)
{
	uORBTest::UnitTest &t = uORBTest::UnitTest::instance();
	return t.pubsublatency_main();
}
