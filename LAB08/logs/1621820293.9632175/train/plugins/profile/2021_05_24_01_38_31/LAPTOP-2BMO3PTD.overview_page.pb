�	V}��b�h@V}��b�h@!V}��b�h@	��O�=}2@��O�=}2@!��O�=}2@"e
=type.googleapis.com/tensorflow.profiler.PerGenericStepDetails$V}��b�h@Gx$(U@AX�2ı�R@YZd;�/B@*	�������@2F
Iterator::Model��?,B@!+����$U@)5�8EG*B@1��SiX"U@:Preprocessing2g
0Iterator::Model::Prefetch::FlatMap[0]::Generators��@!@����.@)s��@1@����.@:Preprocessing2P
Iterator::Model::Prefetch� �	��?!Z|��X�?)� �	��?1Z|��X�?:Preprocessing2Y
"Iterator::Model::Prefetch::FlatMap��4�8�@!�F����.@)-C��6j?1M��~?:Preprocessing:�
]Enqueuing data: you may want to combine small input data chunks into fewer but larger chunks.
�Data preprocessing: you may increase num_parallel_calls in <a href="https://www.tensorflow.org/api_docs/python/tf/data/Dataset#map" target="_blank">Dataset map()</a> or preprocess the data OFFLINE.
�Reading data from files in advance: you may tune parameters in the following tf.data API (<a href="https://www.tensorflow.org/api_docs/python/tf/data/Dataset#prefetch" target="_blank">prefetch size</a>, <a href="https://www.tensorflow.org/api_docs/python/tf/data/Dataset#interleave" target="_blank">interleave cycle_length</a>, <a href="https://www.tensorflow.org/api_docs/python/tf/data/TFRecordDataset#class_tfrecorddataset" target="_blank">reader buffer_size</a>)
�Reading data from files on demand: you should read data IN ADVANCE using the following tf.data API (<a href="https://www.tensorflow.org/api_docs/python/tf/data/Dataset#prefetch" target="_blank">prefetch</a>, <a href="https://www.tensorflow.org/api_docs/python/tf/data/Dataset#interleave" target="_blank">interleave</a>, <a href="https://www.tensorflow.org/api_docs/python/tf/data/TFRecordDataset#class_tfrecorddataset" target="_blank">reader buffer</a>)
�Other data reading or processing: you may consider using the <a href="https://www.tensorflow.org/programmers_guide/datasets" target="_blank">tf.data API</a> (if you are not using it now)�
:type.googleapis.com/tensorflow.profiler.BottleneckAnalysis�
both�Your program is MODERATELY input-bound because 18.5% of the total step time sampled is waiting for input. Therefore, you would need to reduce both the input time and other time.no*high2t43.0 % of the total step time sampled is spent on 'All Others' time. This could be due to Python execution overhead.9��O�=}2@>Look at Section 3 for the breakdown of input time on the host.B�
@type.googleapis.com/tensorflow.profiler.GenericStepTimeBreakdown�
	Gx$(U@Gx$(U@!Gx$(U@      ��!       "      ��!       *      ��!       2	X�2ı�R@X�2ı�R@!X�2ı�R@:      ��!       B      ��!       J	Zd;�/B@Zd;�/B@!Zd;�/B@R      ��!       Z	Zd;�/B@Zd;�/B@!Zd;�/B@JCPU_ONLYY��O�=}2@b Y      Y@qemU�J@"�	
both�Your program is MODERATELY input-bound because 18.5% of the total step time sampled is waiting for input. Therefore, you would need to reduce both the input time and other time.b
`input_pipeline_analyzer (especially Section 3 for the breakdown of input operations on the Host)m
ktrace_viewer (look at the activities on the timeline of each Host Thread near the bottom of the trace view)"T
Rtensorflow_stats (identify the time-consuming operations executed on the CPU_ONLY)"Z
Xtrace_viewer (look at the activities on the timeline of each CPU_ONLY in the trace view)*�
�<a href="https://www.tensorflow.org/guide/data_performance_analysis" target="_blank">Analyze tf.data performance with the TF Profiler</a>*y
w<a href="https://www.tensorflow.org/guide/data_performance" target="_blank">Better performance with the tf.data API</a>2�
=type.googleapis.com/tensorflow.profiler.GenericRecommendation�
nohigh"t43.0 % of the total step time sampled is spent on 'All Others' time. This could be due to Python execution overhead.:
Refer to the TF2 Profiler FAQb�52.0317% of Op time on the host used eager execution. Performance could be improved with <a href="https://www.tensorflow.org/guide/function" target="_blank">tf.function.</a>2"CPU: B 