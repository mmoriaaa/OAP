/*
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The ASF licenses this file to You under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with
 * the License.  You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package org.apache.spark.sql.execution.streaming

import java.io.File
import java.util.UUID

import org.apache.hadoop.conf.Configuration
import org.apache.hadoop.fs.Path
import org.apache.spark.SparkConf
import org.apache.spark.sql.streaming.StreamTest

class StreamMetadataSuite extends StreamTest {

  override def sparkConf: SparkConf =
    super.sparkConf
      .setAppName("test")
      .set("spark.sql.parquet.columnarReaderBatchSize", "4096")
      .set("spark.sql.sources.useV1SourceList", "avro")
      .set("spark.sql.extensions", "com.intel.oap.ColumnarPlugin")
      .set("spark.sql.execution.arrow.maxRecordsPerBatch", "4096")
      //.set("spark.shuffle.manager", "org.apache.spark.shuffle.sort.ColumnarShuffleManager")
      .set("spark.memory.offHeap.enabled", "true")
      .set("spark.memory.offHeap.size", "10m")
      .set("spark.sql.join.preferSortMergeJoin", "false")
      .set("spark.sql.columnar.codegen.hashAggregate", "false")
      .set("spark.oap.sql.columnar.wholestagecodegen", "false")
      .set("spark.sql.columnar.window", "false")
      .set("spark.unsafe.exceptionOnMemoryLeak", "false")
      //.set("spark.sql.columnar.tmp_dir", "/codegen/nativesql/")
      .set("spark.sql.columnar.sort.broadcastJoin", "true")
      .set("spark.oap.sql.columnar.preferColumnar", "true")

  test("writing and reading") {
    withTempDir { dir =>
      val id = UUID.randomUUID.toString
      val metadata = StreamMetadata(id)
      val file = new Path(new File(dir, "test").toString)
      StreamMetadata.write(metadata, file, hadoopConf)
      val readMetadata = StreamMetadata.read(file, hadoopConf)
      assert(readMetadata.nonEmpty)
      assert(readMetadata.get.id === id)
    }
  }

  test("read Spark 2.1.0 format") {
    // query-metadata-logs-version-2.1.0.txt has the execution metadata generated by Spark 2.1.0
    assert(
      readForResource("query-metadata-logs-version-2.1.0.txt") ===
      StreamMetadata("d366a8bf-db79-42ca-b5a4-d9ca0a11d63e"))
  }

  private def readForResource(fileName: String): StreamMetadata = {
    val input = getClass.getResource(s"/structured-streaming/$fileName")
    StreamMetadata.read(new Path(input.toString), hadoopConf).get
  }

  private val hadoopConf = new Configuration()
}
