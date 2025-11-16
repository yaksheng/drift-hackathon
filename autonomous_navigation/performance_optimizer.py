"""
Performance Optimization Module

Optimizes navigation system performance through:
- Frame skipping
- Caching
- Async improvements
- Resource management
"""

import time
import numpy as np
from typing import Optional, Dict, Any
from collections import deque
from dataclasses import dataclass


@dataclass
class PerformanceMetrics:
    """Performance metrics"""
    frame_rate: float = 0.0
    processing_time: float = 0.0
    cache_hit_rate: float = 0.0
    skipped_frames: int = 0
    total_frames: int = 0


class PerformanceOptimizer:
    """
    Performance optimization for navigation system
    
    Implements:
    - Adaptive frame skipping
    - Result caching
    - Processing time tracking
    - Resource management
    """
    
    def __init__(self,
                 target_fps: float = 10.0,
                 max_processing_time: float = 0.1,  # seconds
                 cache_size: int = 10):
        """
        Initialize performance optimizer
        
        Args:
            target_fps: Target frames per second
            max_processing_time: Maximum processing time per frame (seconds)
            cache_size: Size of result cache
        """
        self.target_fps = target_fps
        self.max_processing_time = max_processing_time
        self.frame_interval = 1.0 / target_fps
        
        # Frame timing
        self.last_frame_time = 0.0
        self.frame_times = deque(maxlen=30)  # Track last 30 frames
        
        # Processing time tracking
        self.processing_times = deque(maxlen=30)
        
        # Result caching
        self.cache: Dict[str, Any] = {}
        self.cache_size = cache_size
        self.cache_hits = 0
        self.cache_misses = 0
        
        # Metrics
        self.metrics = PerformanceMetrics()
        
        # Adaptive frame skipping
        self.skip_frames = 0
        self.adaptive_skip = True
    
    def should_process_frame(self, current_time: float) -> bool:
        """
        Determine if current frame should be processed
        
        Uses adaptive frame skipping to maintain target FPS
        
        Args:
            current_time: Current time
            
        Returns:
            True if frame should be processed
        """
        # Always process first frame
        if self.last_frame_time == 0.0:
            self.last_frame_time = current_time
            return True
        
        # Calculate time since last frame
        time_since_last = current_time - self.last_frame_time
        
        # If adaptive skipping is enabled, adjust based on performance
        if self.adaptive_skip:
            # Check if we're running behind
            if len(self.processing_times) > 0:
                avg_processing = sum(self.processing_times) / len(self.processing_times)
                
                # If processing takes too long, skip more frames
                if avg_processing > self.max_processing_time:
                    self.skip_frames = min(3, self.skip_frames + 1)
                elif avg_processing < self.max_processing_time * 0.5:
                    self.skip_frames = max(0, self.skip_frames - 1)
        
        # Skip frames if needed
        if self.skip_frames > 0:
            self.skip_frames -= 1
            self.metrics.skipped_frames += 1
            return False
        
        # Check if enough time has passed for target FPS
        if time_since_last >= self.frame_interval:
            self.last_frame_time = current_time
            return True
        
        return False
    
    def start_processing(self) -> float:
        """
        Start processing timer
        
        Returns:
            Start time
        """
        return time.time()
    
    def end_processing(self, start_time: float):
        """
        End processing timer and update metrics
        
        Args:
            start_time: Start time from start_processing()
        """
        processing_time = time.time() - start_time
        self.processing_times.append(processing_time)
        self.metrics.processing_time = processing_time
        
        # Update frame rate
        if len(self.frame_times) > 0:
            intervals = [
                self.frame_times[i] - self.frame_times[i-1]
                for i in range(1, len(self.frame_times))
            ]
            if intervals:
                avg_interval = sum(intervals) / len(intervals)
                self.metrics.frame_rate = 1.0 / avg_interval if avg_interval > 0 else 0.0
    
    def record_frame(self, current_time: float):
        """Record frame time for FPS calculation"""
        self.frame_times.append(current_time)
        self.metrics.total_frames += 1
    
    def get_cached(self, key: str) -> Optional[Any]:
        """
        Get cached result
        
        Args:
            key: Cache key
            
        Returns:
            Cached value or None
        """
        if key in self.cache:
            self.cache_hits += 1
            return self.cache[key]
        else:
            self.cache_misses += 1
            return None
    
    def set_cached(self, key: str, value: Any):
        """
        Cache a result
        
        Args:
            key: Cache key
            value: Value to cache
        """
        # Limit cache size
        if len(self.cache) >= self.cache_size:
            # Remove oldest entry (simple FIFO)
            oldest_key = next(iter(self.cache))
            del self.cache[oldest_key]
        
        self.cache[key] = value
    
    def clear_cache(self):
        """Clear all cached results"""
        self.cache.clear()
    
    def get_metrics(self) -> PerformanceMetrics:
        """
        Get current performance metrics
        
        Returns:
            PerformanceMetrics object
        """
        # Update cache hit rate
        total_requests = self.cache_hits + self.cache_misses
        if total_requests > 0:
            self.metrics.cache_hit_rate = self.cache_hits / total_requests
        else:
            self.metrics.cache_hit_rate = 0.0
        
        return self.metrics
    
    def reset_metrics(self):
        """Reset performance metrics"""
        self.metrics = PerformanceMetrics()
        self.cache_hits = 0
        self.cache_misses = 0
        self.frame_times.clear()
        self.processing_times.clear()
    
    def optimize_for_low_power(self):
        """Optimize for low power consumption"""
        # Increase frame skipping
        self.skip_frames = 2
        self.target_fps = 5.0
        self.frame_interval = 1.0 / self.target_fps
    
    def optimize_for_performance(self):
        """Optimize for maximum performance"""
        # Reduce frame skipping
        self.skip_frames = 0
        self.target_fps = 15.0
        self.frame_interval = 1.0 / self.target_fps

