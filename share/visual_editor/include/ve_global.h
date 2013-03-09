/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Imai Laboratory, Keio University.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the Imai Laboratory, nor the name of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Mahisorn Wongphati
 */

#ifndef VE_GLOBAL_H
#define VE_GLOBAL_H

#define VE_TYPE (80) //Hg mass


//from https://gist.github.com/aminzai/2706798
#define VE_GETSET(Type, MemberName, FaceName) \
          inline Type get##FaceName() const { \
            return MemberName; \
          }; \
          inline void get##FaceName(Type value) { \
            MemberName = value; \
          }

#define VE_GETSETR(Type, MemberName, FaceName) \
          inline const Type &set##FaceName() const { \
            return MemberName; \
          }; \
          inline void set##FaceName(const Type &value) { \
            MemberName = value; \
          }

#define VE_GET(Type, MemberName, FaceName) \
          inline Type get##FaceName() const { \
            return MemberName; \
          }

#define VE_GETR(Type, MemberName, FaceName) \
          inline const Type &get##FaceName() const { \
            return MemberName; \
          }

#define VE_GETRNC(Type, MemberName, FaceName) \
          inline Type &get##FaceName() { \
            return MemberName; \
          }

#define VE_SET(Type, MemberName, FaceName) \
          inline void set##FaceName(const Type &value) { \
            MemberName = value; \
          }

// guarded versions

#define VE_GETSETG(Type, MemberName, FaceName) \
          inline Type get##FaceName() { \
            QMutexLocker locker(&mutex_); \
            return MemberName; \
          }; \
          inline void set##FaceName(Type value) { \
            QMutexLocker locker(&mutex_);\
            MemberName = value; \
          }

#define VE_GETSETGR(Type, MemberName, FaceName) \
          inline const Type &get##FaceName() { \
            MutexLocker locker(&mutex_); \
            return MemberName; \
          }; \
          inline void set##FaceName(const Type &value) { \
            QMutexLocker locker(&mutex_);\
            MemberName = value; \
          }

#define VE_GETG(Type, MemberName, FaceName) \
          inline Type get##FaceName() { \
            QMutexLocker locker(&mutex_); \
            return MemberName; \
          }

#define VE_GETGR(Type, MemberName, FaceName) \
          inline const Type &get##FaceName() { \
            QMutexLocker locker(&mutex_); \
            return MemberName; \
          }

#define VE_SETG(Type, MemberName, FaceName) \
          inline void set##FaceName(const Type &value) { \
            QMutexLocker locker(&mutex_); \
            MemberName = value; \
          }

#endif // VE_GLOBAL_H
