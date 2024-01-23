/*
 * SYNTIANT CONFIDENTIAL
 *
 * _____________________
 *
 * Copyright (c) 2017-2020 Syntiant Corporation
 * All Rights Reserved.
 *
 * NOTICE:  All information contained herein is, and remains the property of
 * Syntiant Corporation and its suppliers, if any.  The intellectual and
 * technical concepts contained herein are proprietary to Syntiant Corporation
 * and its suppliers and may be covered by U.S. and Foreign Patents, patents in
 * process, and are protected by trade secret or copyright law.  Dissemination
 * of this information or reproduction of this material is strictly forbidden
 * unless prior written permission is obtained from Syntiant Corporation.
 *
 */
#ifndef NDP10X_ASSERT_H
#define NDP10X_ASSERT_H

/* compile-time assert */
#define CASSERT(predicate, file) _impl_CASSERT_LINE(predicate, __LINE__, file)
#define _impl_PASTE(a, b) a##b
#define _impl_CASSERT_LINE(predicate, line, file)                              \
    typedef char _impl_PASTE(                                                  \
        assertion_failed_##file##_, line)[2 * !!(predicate)-1];

#endif
