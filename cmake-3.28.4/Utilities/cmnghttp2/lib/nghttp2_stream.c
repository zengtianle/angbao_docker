/*
 * nghttp2 - HTTP/2 C Library
 *
 * Copyright (c) 2012 Tatsuhiro Tsujikawa
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#include "nghttp2_stream.h"

#include <assert.h>
#include <stdio.h>

#include "nghttp2_session.h"
#include "nghttp2_helper.h"
#include "nghttp2_debug.h"
#include "nghttp2_frame.h"

/* Maximum distance between any two stream's cycle in the same
   priority queue.  Imagine stream A's cycle is A, and stream B's
   cycle is B, and A < B.  The cycle is unsigned 32 bit integer, it
   may get overflow.  Because of how we calculate the next cycle
   value, if B - A is less than or equals to
   NGHTTP2_MAX_CYCLE_DISTANCE, A and B are in the same scale, in other
   words, B is really greater than or equal to A.  Otherwise, A is a
   result of overflow, and it is actually A > B if we consider that
   fact. */
#define NGHTTP2_MAX_CYCLE_DISTANCE                                             \
  ((uint64_t)NGHTTP2_MAX_FRAME_SIZE_MAX * 256 + 255)

static int stream_less(const void *lhsx, const void *rhsx) {
  const nghttp2_stream *lhs, *rhs;

  lhs = nghttp2_struct_of(lhsx, nghttp2_stream, pq_entry);
  rhs = nghttp2_struct_of(rhsx, nghttp2_stream, pq_entry);

  if (lhs->cycle == rhs->cycle) {
    return lhs->seq < rhs->seq;
  }

  return rhs->cycle - lhs->cycle <= NGHTTP2_MAX_CYCLE_DISTANCE;
}

void nghttp2_stream_init(nghttp2_stream *stream, int32_t stream_id,
                         uint8_t flags, nghttp2_stream_state initial_state,
                         int32_t weight, int32_t remote_initial_window_size,
                         int32_t local_initial_window_size,
                         void *stream_user_data, nghttp2_mem *mem) {
  nghttp2_pq_init(&stream->obq, stream_less, mem);

  stream->stream_id = stream_id;
  stream->flags = flags;
  stream->state = initial_state;
  stream->shut_flags = NGHTTP2_SHUT_NONE;
  stream->stream_user_data = stream_user_data;
  stream->item = NULL;
  stream->remote_window_size = remote_initial_window_size;
  stream->local_window_size = local_initial_window_size;
  stream->recv_window_size = 0;
  stream->consumed_size = 0;
  stream->recv_reduction = 0;
  stream->window_update_queued = 0;

  stream->dep_prev = NULL;
  stream->dep_next = NULL;
  stream->sib_prev = NULL;
  stream->sib_next = NULL;

  stream->closed_prev = NULL;
  stream->closed_next = NULL;

  stream->weight = weight;
  stream->sum_dep_weight = 0;

  stream->http_flags = NGHTTP2_HTTP_FLAG_NONE;
  stream->content_length = -1;
  stream->recv_content_length = 0;
  stream->status_code = -1;

  stream->queued = 0;
  stream->descendant_last_cycle = 0;
  stream->cycle = 0;
  stream->pending_penalty = 0;
  stream->descendant_next_seq = 0;
  stream->seq = 0;
  stream->last_writelen = 0;

  stream->extpri = stream->http_extpri = NGHTTP2_EXTPRI_DEFAULT_URGENCY;
}

void nghttp2_stream_free(nghttp2_stream *stream) {
  nghttp2_pq_free(&stream->obq);
  /* We don't free stream->item.  If it is assigned to aob, then
     active_outbound_item_reset() will delete it.  Otherwise,
     nghttp2_stream_close() or session_del() will delete it. */
}

void nghttp2_stream_shutdown(nghttp2_stream *stream, nghttp2_shut_flag flag) {
  stream->shut_flags = (uint8_t)(stream->shut_flags | flag);
}

/*
 * Returns nonzero if |stream| is active.  This function does not take
 * into account its descendants.
 */
static int stream_active(nghttp2_stream *stream) {
  return stream->item &&
         (stream->flags & NGHTTP2_STREAM_FLAG_DEFERRED_ALL) == 0;
}

/*
 * Returns nonzero if |stream| or one of its descendants is active
 */
static int stream_subtree_active(nghttp2_stream *stream) {
  return stream_active(stream) || !nghttp2_pq_empty(&stream->obq);
}

/*
 * Returns next cycle for |stream|.
 */
static void stream_next_cycle(nghttp2_stream *stream, uint64_t last_cycle) {
  uint64_t penalty;

  penalty = (uint64_t)stream->last_writelen * NGHTTP2_MAX_WEIGHT +
            stream->pending_penalty;

  stream->cycle = last_cycle + penalty / (uint32_t)stream->weight;
  stream->pending_penalty = (uint32_t)(penalty % (uint32_t)stream->weight);
}

static int stream_obq_push(nghttp2_stream *dep_stream, nghttp2_stream *stream) {
  int rv;

  for (; dep_stream && !stream->queued;
       stream = dep_stream, dep_stream = dep_stream->dep_prev) {
    stream_next_cycle(stream, dep_stream->descendant_last_cycle);
    stream->seq = dep_stream->descendant_next_seq++;

    DEBUGF("stream: stream=%d obq push cycle=%lu\n", stream->stream_id,
           stream->cycle);

    DEBUGF("stream: push stream %d to stream %d\n", stream->stream_id,
           dep_stream->stream_id);

    rv = nghttp2_pq_push(&dep_stream->obq, &stream->pq_entry);
    if (rv != 0) {
      return rv;
    }
    stream->queued = 1;
  }

  return 0;
}

/*
 * Removes |stream| from parent's obq.  If removal of |stream| makes
 * parent's obq empty, and parent is not active, then parent is also
 * removed.  This process is repeated recursively.
 */
static void stream_obq_remove(nghttp2_stream *stream) {
  nghttp2_stream *dep_stream;

  dep_stream = stream->dep_prev;

  if (!stream->queued) {
    return;
  }

  for (; dep_stream; stream = dep_stream, dep_stream = dep_stream->dep_prev) {
    DEBUGF("stream: remove stream %d from stream %d\n", stream->stream_id,
           dep_stream->stream_id);

    nghttp2_pq_remove(&dep_stream->obq, &stream->pq_entry);

    assert(stream->queued);

    stream->queued = 0;
    stream->cycle = 0;
    stream->pending_penalty = 0;
    stream->descendant_last_cycle = 0;
    stream->last_writelen = 0;

    if (stream_subtree_active(dep_stream)) {
      return;
    }
  }
}

/*
 * Moves |stream| from |src|'s obq to |dest|'s obq.  Removal from
 * |src|'s obq is just done calling nghttp2_pq_remove(), so it does
 * not recursively remove |src| and ancestors, like
 * stream_obq_remove().
 */
static int stream_obq_move(nghttp2_stream *dest, nghttp2_stream *src,
                           nghttp2_stream *stream) {
  if (!stream->queued) {
    return 0;
  }

  DEBUGF("stream: remove stream %d from stream %d (move)\n", stream->stream_id,
         src->stream_id);

  nghttp2_pq_remove(&src->obq, &stream->pq_entry);
  stream->queued = 0;

  return stream_obq_push(dest, stream);
}

void nghttp2_stream_reschedule(nghttp2_stream *stream) {
  nghttp2_stream *dep_stream;

  assert(stream->queued);

  dep_stream = stream->dep_prev;

  for (; dep_stream; stream = dep_stream, dep_stream = dep_stream->dep_prev) {
    nghttp2_pq_remove(&dep_stream->obq, &stream->pq_entry);

    stream_next_cycle(stream, dep_stream->descendant_last_cycle);
    stream->seq = dep_stream->descendant_next_seq++;

    nghttp2_pq_push(&dep_stream->obq, &stream->pq_entry);

    DEBUGF("stream: stream=%d obq resched cycle=%lu\n", stream->stream_id,
           stream->cycle);

    dep_stream->last_writelen = stream->last_writelen;
  }
}

void nghttp2_stream_change_weight(nghttp2_stream *stream, int32_t weight) {
  nghttp2_stream *dep_stream;
  uint64_t last_cycle;
  int32_t old_weight;
  uint64_t wlen_penalty;

  if (stream->weight == weight) {
    return;
  }

  old_weight = stream->weight;
  stream->weight = weight;

  dep_stream = stream->dep_prev;

  if (!dep_stream) {
    return;
  }

  dep_stream->sum_dep_weight += weight - old_weight;

  if (!stream->queued) {
    return;
  }

  nghttp2_pq_remove(&dep_stream->obq, &stream->pq_entry);

  wlen_penalty = (uint64_t)stream->last_writelen * NGHTTP2_MAX_WEIGHT;

  /* Compute old stream->pending_penalty we used to calculate
     stream->cycle */
  stream->pending_penalty =
      (uint32_t)((stream->pending_penalty + (uint32_t)old_weight -
                  (wlen_penalty % (uint32_t)old_weight)) %
                 (uint32_t)old_weight);

  last_cycle = stream->cycle -
               (wlen_penalty + stream->pending_penalty) / (uint32_t)old_weight;

  /* Now we have old stream->pending_penalty and new stream->weight in
     place */
  stream_next_cycle(stream, last_cycle);

  if (dep_stream->descendant_last_cycle - stream->cycle <=
      NGHTTP2_MAX_CYCLE_DISTANCE) {
    stream->cycle = dep_stream->descendant_last_cycle;
  }

  /* Continue to use same stream->seq */

  nghttp2_pq_push(&dep_stream->obq, &stream->pq_entry);

  DEBUGF("stream: stream=%d obq resched cycle=%lu\n", stream->stream_id,
         stream->cycle);
}

static nghttp2_stream *stream_last_sib(nghttp2_stream *stream) {
  for (; stream->sib_next; stream = stream->sib_next)
    ;

  return stream;
}

int32_t nghttp2_stream_dep_distributed_weight(nghttp2_stream *stream,
                                              int32_t weight) {
  weight = stream->weight * weight / stream->sum_dep_weight;

  return nghttp2_max(1, weight);
}

#ifdef STREAM_DEP_DEBUG

static void ensure_inactive(nghttp2_stream *stream) {
  nghttp2_stream *si;

  if (stream->queued) {
    fprintf(stderr, "stream(%p)=%d, stream->queued = 1; want 0\n", stream,
            stream->stream_id);
    assert(0);
  }

  if (stream_active(stream)) {
    fprintf(stderr, "stream(%p)=%d, stream_active(stream) = 1; want 0\n",
            stream, stream->stream_id);
    assert(0);
  }

  if (!nghttp2_pq_empty(&stream->obq)) {
    fprintf(stderr, "stream(%p)=%d, nghttp2_pq_size() = %zu; want 0\n", stream,
            stream->stream_id, nghttp2_pq_size(&stream->obq));
    assert(0);
  }

  for (si = stream->dep_next; si; si = si->sib_next) {
    ensure_inactive(si);
  }
}

static void check_queued(nghttp2_stream *stream) {
  nghttp2_stream *si;
  int queued;

  if (stream->queued) {
    if (!stream_subtree_active(stream)) {
      fprintf(stderr,
              "stream(%p)=%d, stream->queued == 1, but "
              "stream_active() == %d and nghttp2_pq_size(&stream->obq) = %zu\n",
              stream, stream->stream_id, stream_active(stream),
              nghttp2_pq_size(&stream->obq));
      assert(0);
    }
    if (!stream_active(stream)) {
      queued = 0;
      for (si = stream->dep_next; si; si = si->sib_next) {
        if (si->queued) {
          ++queued;
        }
      }
      if (queued == 0) {
        fprintf(stderr,
                "stream(%p)=%d, stream->queued == 1, and "
                "!stream_active(), but no descendants is queued\n",
                stream, stream->stream_id);
        assert(0);
      }
    }

    for (si = stream->dep_next; si; si = si->sib_next) {
      check_queued(si);
    }
  } else {
    if (stream_active(stream) || !nghttp2_pq_empty(&stream->obq)) {
      fprintf(stderr,
              "stream(%p) = %d, stream->queued == 0, but "
              "stream_active(stream) == %d and "
              "nghttp2_pq_size(&stream->obq) = %zu\n",
              stream, stream->stream_id, stream_active(stream),
              nghttp2_pq_size(&stream->obq));
      assert(0);
    }
    for (si = stream->dep_next; si; si = si->sib_next) {
      ensure_inactive(si);
    }
  }
}

static void check_sum_dep(nghttp2_stream *stream) {
  nghttp2_stream *si;
  int32_t n = 0;
  for (si = stream->dep_next; si; si = si->sib_next) {
    n += si->weight;
  }
  if (n != stream->sum_dep_weight) {
    fprintf(stderr, "stream(%p)=%d, sum_dep_weight = %d; want %d\n", stream,
            stream->stream_id, n, stream->sum_dep_weight);
    assert(0);
  }
  for (si = stream->dep_next; si; si = si->sib_next) {
    check_sum_dep(si);
  }
}

static void check_dep_prev(nghttp2_stream *stream) {
  nghttp2_stream *si;
  for (si = stream->dep_next; si; si = si->sib_next) {
    if (si->dep_prev != stream) {
      fprintf(stderr, "si->dep_prev = %p; want %p\n", si->dep_prev, stream);
      assert(0);
    }
    check_dep_prev(si);
  }
}

#endif /* STREAM_DEP_DEBUG */

#ifdef STREAM_DEP_DEBUG
static void validate_tree(nghttp2_stream *stream) {
  nghttp2_stream *si;

  if (!stream) {
    return;
  }

  for (; stream->dep_prev; stream = stream->dep_prev)
    ;

  assert(stream->stream_id == 0);
  assert(!stream->queued);

  fprintf(stderr, "checking...\n");
  if (nghttp2_pq_empty(&stream->obq)) {
    fprintf(stderr, "root obq empty\n");
    for (si = stream->dep_next; si; si = si->sib_next) {
      ensure_inactive(si);
    }
  } else {
    for (si = stream->dep_next; si; si = si->sib_next) {
      check_queued(si);
    }
  }

  check_sum_dep(stream);
  check_dep_prev(stream);
}
#else  /* !STREAM_DEP_DEBUG */
static void validate_tree(nghttp2_stream *stream) { (void)stream; }
#endif /* !STREAM_DEP_DEBUG*/

static int stream_update_dep_on_attach_item(nghttp2_stream *stream) {
  int rv;

  rv = stream_obq_push(stream->dep_prev, stream);
  if (rv != 0) {
    return rv;
  }

  validate_tree(stream);
  return 0;
}

static int stream_update_dep_on_detach_item(nghttp2_stream *stream) {
  if (nghttp2_pq_empty(&stream->obq)) {
    stream_obq_remove(stream);
  }

  validate_tree(stream);

  return 0;
}

int nghttp2_stream_attach_item(nghttp2_stream *stream,
                               nghttp2_outbound_item *item) {
  int rv;

  assert((stream->flags & NGHTTP2_STREAM_FLAG_DEFERRED_ALL) == 0);
  assert(stream->item == NULL);

  DEBUGF("stream: stream=%d attach item=%p\n", stream->stream_id, item);

  stream->item = item;

  if (stream->flags & NGHTTP2_STREAM_FLAG_NO_RFC7540_PRIORITIES) {
    return 0;
  }

  rv = stream_update_dep_on_attach_item(stream);
  if (rv != 0) {
    /* This may relave stream->queued == 1, but stream->item == NULL.
       But only consequence of this error is fatal one, and session
       destruction.  In that execution path, these inconsistency does
       not matter. */
    stream->item = NULL;
    return rv;
  }

  return 0;
}

int nghttp2_stream_detach_item(nghttp2_stream *stream) {
  DEBUGF("stream: stream=%d detach item=%p\n", stream->stream_id, stream->item);

  stream->item = NULL;
  stream->flags = (uint8_t)(stream->flags & ~NGHTTP2_STREAM_FLAG_DEFERRED_ALL);

  if (stream->flags & NGHTTP2_STREAM_FLAG_NO_RFC7540_PRIORITIES) {
    return 0;
  }

  return stream_update_dep_on_detach_item(stream);
}

int nghttp2_stream_defer_item(nghttp2_stream *stream, uint8_t flags) {
  assert(stream->item);

  DEBUGF("stream: stream=%d defer item=%p cause=%02x\n", stream->stream_id,
         stream->item, flags);

  stream->flags |= flags;

  if (stream->flags & NGHTTP2_STREAM_FLAG_NO_RFC7540_PRIORITIES) {
    return 0;
  }

  return stream_update_dep_on_detach_item(stream);
}

int nghttp2_stream_resume_deferred_item(nghttp2_stream *stream, uint8_t flags) {
  assert(stream->item);

  DEBUGF("stream: stream=%d resume item=%p flags=%02x\n", stream->stream_id,
         stream->item, flags);

  stream->flags = (uint8_t)(stream->flags & ~flags);

  if (stream->flags & NGHTTP2_STREAM_FLAG_DEFERRED_ALL) {
    return 0;
  }

  if (stream->flags & NGHTTP2_STREAM_FLAG_NO_RFC7540_PRIORITIES) {
    return 0;
  }

  return stream_update_dep_on_attach_item(stream);
}

int nghttp2_stream_check_deferred_item(nghttp2_stream *stream) {
  return stream->item && (stream->flags & NGHTTP2_STREAM_FLAG_DEFERRED_ALL);
}

int nghttp2_stream_check_deferred_by_flow_control(nghttp2_stream *stream) {
  return stream->item &&
         (stream->flags & NGHTTP2_STREAM_FLAG_DEFERRED_FLOW_CONTROL);
}

static int update_initial_window_size(int32_t *window_size_ptr,
                                      int32_t new_initial_window_size,
                                      int32_t old_initial_window_size) {
  int64_t new_window_size = (int6