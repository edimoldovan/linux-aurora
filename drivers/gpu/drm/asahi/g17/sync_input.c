// SPDX-License-Identifier: GPL-2.0-only OR MIT
#include <linux/dma-fence.h>
#include <linux/dma-fence-unwrap.h>
#include <linux/errno.h>
#include "sync_input.h"

/* Called without the device mutex: status polling may run fence callbacks. */
int asahi_g17_collect_fences(struct dma_fence *head,
			   struct asahi_g17_fence_leaf *out, size_t capacity)
{
	struct dma_fence_unwrap cursor;
	struct dma_fence *leaf;
	size_t count = 0;
	int error = 0;

	dma_fence_unwrap_for_each(leaf, &cursor, head) {
		if (dma_fence_is_container(leaf)) {
			error = -EOPNOTSUPP;
			break;
		}
		if (count == capacity) {
			error = -ENOSPC;
			break;
		}
		out[count].fence = dma_fence_get(leaf);
		out[count].status = dma_fence_get_status(leaf);
		++count;
	}
	/* A bounded early exit still owns the iterator's current chain. */
	dma_fence_put(cursor.chain);
	if (error) {
		while (count)
			dma_fence_put(out[--count].fence);
		return error;
	}
	return count;
}

void asahi_g17_put_fence(struct dma_fence *fence)
{
	dma_fence_put(fence);
}
