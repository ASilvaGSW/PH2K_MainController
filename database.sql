-- MySQL dump 10.13  Distrib 5.7.17, for Win64 (x86_64)
--
-- Host: 127.0.0.1    Database: maincontroller
-- ------------------------------------------------------
-- Server version	5.5.5-10.3.9-MariaDB

/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!40101 SET NAMES utf8 */;
/*!40103 SET @OLD_TIME_ZONE=@@TIME_ZONE */;
/*!40103 SET TIME_ZONE='+00:00' */;
/*!40014 SET @OLD_UNIQUE_CHECKS=@@UNIQUE_CHECKS, UNIQUE_CHECKS=0 */;
/*!40014 SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0 */;
/*!40101 SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='NO_AUTO_VALUE_ON_ZERO' */;
/*!40111 SET @OLD_SQL_NOTES=@@SQL_NOTES, SQL_NOTES=0 */;

--
-- Table structure for table `device_functions`
--

DROP TABLE IF EXISTS `device_functions`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `device_functions` (
  `id` int(11) NOT NULL AUTO_INCREMENT,
  `device_type_id` int(11) NOT NULL,
  `function_id` int(11) NOT NULL,
  `description` varchar(200) NOT NULL,
  `bit0` tinyint(1) DEFAULT NULL,
  `bit1` tinyint(1) DEFAULT NULL,
  `bit2` tinyint(1) DEFAULT NULL,
  `bit3` tinyint(1) DEFAULT NULL,
  `bit4` tinyint(1) DEFAULT NULL,
  `bit5` tinyint(1) DEFAULT NULL,
  `bit6` tinyint(1) DEFAULT NULL,
  `bit7` tinyint(1) DEFAULT NULL,
  `created_at` datetime DEFAULT NULL,
  `active` tinyint(1) DEFAULT NULL,
  PRIMARY KEY (`id`),
  KEY `device_type_id` (`device_type_id`),
  CONSTRAINT `device_functions_ibfk_1` FOREIGN KEY (`device_type_id`) REFERENCES `device_types` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=2 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `device_functions`
--

LOCK TABLES `device_functions` WRITE;
/*!40000 ALTER TABLE `device_functions` DISABLE KEYS */;
INSERT INTO `device_functions` VALUES (1,1,1,'Reset 1',1,0,0,0,0,0,0,0,'2025-08-12 00:00:00',1);
/*!40000 ALTER TABLE `device_functions` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `device_types`
--

DROP TABLE IF EXISTS `device_types`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `device_types` (
  `id` int(11) NOT NULL AUTO_INCREMENT,
  `name` varchar(50) NOT NULL,
  `description` text DEFAULT NULL,
  `created_by` varchar(80) NOT NULL,
  `created_at` datetime DEFAULT NULL,
  `updated_at` datetime DEFAULT NULL,
  `active` tinyint(1) DEFAULT NULL,
  PRIMARY KEY (`id`),
  UNIQUE KEY `name` (`name`)
) ENGINE=InnoDB AUTO_INCREMENT=4 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `device_types`
--

LOCK TABLES `device_types` WRITE;
/*!40000 ALTER TABLE `device_types` DISABLE KEYS */;
INSERT INTO `device_types` VALUES (1,'Camera','AI Camera','asilva','2025-08-12 14:09:26','2025-08-12 14:09:26',1),(2,'Gantry','XYZ Linear Actuator Combo','asilva','2025-08-12 18:34:55','2025-08-12 18:34:55',1),(3,'Pick and Place','Pick and Place X and Z ','asilva','2025-08-12 18:35:30','2025-08-12 18:35:30',1);
/*!40000 ALTER TABLE `device_types` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `devices`
--

DROP TABLE IF EXISTS `devices`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `devices` (
  `id` int(11) NOT NULL AUTO_INCREMENT,
  `name` varchar(100) NOT NULL,
  `canbus_id` varchar(10) NOT NULL,
  `device_type_id` int(11) NOT NULL,
  `created_by` varchar(80) NOT NULL,
  `created_at` datetime DEFAULT NULL,
  `updated_at` datetime DEFAULT NULL,
  `active` tinyint(1) DEFAULT NULL,
  PRIMARY KEY (`id`),
  UNIQUE KEY `canbus_id` (`canbus_id`)
) ENGINE=InnoDB AUTO_INCREMENT=7 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `devices`
--

LOCK TABLES `devices` WRITE;
/*!40000 ALTER TABLE `devices` DISABLE KEYS */;
INSERT INTO `devices` VALUES (1,'Inspection Camera 02','0x190',1,'asilva','2025-08-12 13:44:08','2025-08-12 18:40:15',1),(2,'Gantry 01','0x002',2,'asilva','2025-08-12 18:36:42','2025-08-12 18:36:42',1),(3,'Gantry 02','0x102',2,'asilva','2025-08-12 18:37:06','2025-08-12 18:37:06',1),(4,'Inspection Camera 01','0x090',1,'asilva','2025-08-12 18:37:23','2025-08-12 18:37:23',1),(5,'Pick and Place 01','0x010',3,'asilva','2025-08-12 18:41:01','2025-08-12 18:41:01',1),(6,'Pick and Place 02','0x110',3,'asilva','2025-08-12 18:41:16','2025-08-12 18:41:16',1);
/*!40000 ALTER TABLE `devices` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `insertions`
--

DROP TABLE IF EXISTS `insertions`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `insertions` (
  `id` int(11) NOT NULL AUTO_INCREMENT,
  `part_number_id` int(11) NOT NULL,
  `side` varchar(1) NOT NULL,
  `component` varchar(100) NOT NULL,
  `active` tinyint(1) DEFAULT NULL,
  PRIMARY KEY (`id`),
  KEY `part_number_id` (`part_number_id`),
  CONSTRAINT `insertions_ibfk_1` FOREIGN KEY (`part_number_id`) REFERENCES `part_numbers` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=14 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `insertions`
--

LOCK TABLES `insertions` WRITE;
/*!40000 ALTER TABLE `insertions` DISABLE KEYS */;
INSERT INTO `insertions` VALUES (1,1,'A','A214532',0),(2,1,'B','A142345',0),(3,1,'A','A214532',0),(4,1,'B','A142345',0),(5,2,'A','Nozzle',0),(6,2,'B','Joint',0),(7,3,'B','Nozzle A',1),(8,1,'A','Joint A',0),(9,4,'B','Nozzle A',0),(10,4,'B','Nozzle A',1),(11,5,'A','Joint A',1),(12,5,'B','Nozzle A',1),(13,1,'A','Joint A',1);
/*!40000 ALTER TABLE `insertions` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `joints`
--

DROP TABLE IF EXISTS `joints`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `joints` (
  `id` int(11) NOT NULL AUTO_INCREMENT,
  `model` varchar(100) NOT NULL,
  `description` varchar(255) DEFAULT NULL,
  `created_by` varchar(100) NOT NULL,
  `created_at` datetime NOT NULL,
  `updated_at` datetime NOT NULL,
  `active` tinyint(1) DEFAULT NULL,
  PRIMARY KEY (`id`),
  UNIQUE KEY `model` (`model`)
) ENGINE=InnoDB AUTO_INCREMENT=2 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `joints`
--

LOCK TABLES `joints` WRITE;
/*!40000 ALTER TABLE `joints` DISABLE KEYS */;
INSERT INTO `joints` VALUES (1,'Joint A','Joint A','asilva','2025-08-11 21:17:54','2025-08-11 21:17:54',1);
/*!40000 ALTER TABLE `joints` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `nozzles`
--

DROP TABLE IF EXISTS `nozzles`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `nozzles` (
  `id` int(11) NOT NULL AUTO_INCREMENT,
  `model` varchar(100) NOT NULL,
  `description` varchar(255) DEFAULT NULL,
  `created_by` varchar(100) NOT NULL,
  `created_at` datetime NOT NULL,
  `updated_at` datetime NOT NULL,
  `active` tinyint(1) DEFAULT NULL,
  PRIMARY KEY (`id`),
  UNIQUE KEY `model` (`model`)
) ENGINE=InnoDB AUTO_INCREMENT=2 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `nozzles`
--

LOCK TABLES `nozzles` WRITE;
/*!40000 ALTER TABLE `nozzles` DISABLE KEYS */;
INSERT INTO `nozzles` VALUES (1,'Nozzle A','Nozzle A','asilva','2025-08-11 21:17:16','2025-08-11 21:17:16',1);
/*!40000 ALTER TABLE `nozzles` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `part_numbers`
--

DROP TABLE IF EXISTS `part_numbers`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `part_numbers` (
  `id` int(11) NOT NULL AUTO_INCREMENT,
  `part_number` varchar(100) NOT NULL,
  `name` varchar(200) NOT NULL,
  `created_by` varchar(100) NOT NULL,
  `created_at` datetime NOT NULL,
  `updated_at` datetime NOT NULL,
  `active` tinyint(1) DEFAULT NULL,
  `total_length` int(11) NOT NULL DEFAULT 10,
  PRIMARY KEY (`id`),
  UNIQUE KEY `part_number` (`part_number`)
) ENGINE=InnoDB AUTO_INCREMENT=6 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `part_numbers`
--

LOCK TABLES `part_numbers` WRITE;
/*!40000 ALTER TABLE `part_numbers` DISABLE KEYS */;
INSERT INTO `part_numbers` VALUES (1,'AN-2025-RND-M','RND Sample','asilva','2025-08-11 20:19:00','2025-08-12 15:11:42',1,1000),(2,'TestA','TestA','asilva','2025-08-11 20:44:38','2025-08-11 21:19:27',0,1500),(3,'SamplePart','Sample PArt','asilva','2025-08-11 22:02:41','2025-08-11 22:02:41',1,300),(4,'AN-2025-RND-M2','Sample PArt','asilva','2025-08-11 22:18:22','2025-08-11 22:18:50',1,900),(5,'AN-2025-RND-M3','Test','asilva','2025-08-12 15:11:17','2025-08-12 15:11:17',1,800);
/*!40000 ALTER TABLE `part_numbers` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `prod_his`
--

DROP TABLE IF EXISTS `prod_his`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `prod_his` (
  `id` int(11) NOT NULL AUTO_INCREMENT,
  `machine` varchar(100) NOT NULL,
  `date` date NOT NULL,
  `part_number` varchar(100) NOT NULL,
  `qty` int(11) NOT NULL,
  `timestamp` datetime NOT NULL,
  `user` varchar(100) NOT NULL,
  `deleted` tinyint(1) DEFAULT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=21 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `prod_his`
--

LOCK TABLES `prod_his` WRITE;
/*!40000 ALTER TABLE `prod_his` DISABLE KEYS */;
/*!40000 ALTER TABLE `prod_his` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `stamps`
--

DROP TABLE IF EXISTS `stamps`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `stamps` (
  `id` int(11) NOT NULL AUTO_INCREMENT,
  `part_number_id` int(11) NOT NULL,
  `color` varchar(50) NOT NULL,
  `position` float NOT NULL,
  `active` tinyint(1) DEFAULT NULL,
  PRIMARY KEY (`id`),
  KEY `part_number_id` (`part_number_id`),
  CONSTRAINT `stamps_ibfk_1` FOREIGN KEY (`part_number_id`) REFERENCES `part_numbers` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=12 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `stamps`
--

LOCK TABLES `stamps` WRITE;
/*!40000 ALTER TABLE `stamps` DISABLE KEYS */;
INSERT INTO `stamps` VALUES (1,1,'Red',150,0),(2,1,'Red',300,0),(3,2,'White',300,0),(4,1,'red',300,0),(5,3,'white',150,1),(6,1,'red',300,0),(7,4,'red',250,0),(8,4,'red',250,1),(9,4,'red',350,1),(10,5,'white',500,1),(11,1,'red',300,1);
/*!40000 ALTER TABLE `stamps` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `tapes`
--

DROP TABLE IF EXISTS `tapes`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `tapes` (
  `id` int(11) NOT NULL AUTO_INCREMENT,
  `part_number_id` int(11) NOT NULL,
  `color` varchar(50) NOT NULL,
  `width` float NOT NULL,
  `position` float NOT NULL,
  `active` tinyint(1) DEFAULT NULL,
  PRIMARY KEY (`id`),
  KEY `part_number_id` (`part_number_id`),
  CONSTRAINT `tapes_ibfk_1` FOREIGN KEY (`part_number_id`) REFERENCES `part_numbers` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=11 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `tapes`
--

LOCK TABLES `tapes` WRITE;
/*!40000 ALTER TABLE `tapes` DISABLE KEYS */;
INSERT INTO `tapes` VALUES (1,1,'Yellow',19,150,0),(2,1,'White',19,800,0),(3,2,'Red',19,800,0),(4,1,'red',19,800,0),(5,1,'red',19,800,0),(6,4,'green',19,450,0),(7,4,'green',19,450,1),(8,5,'green',19,250,1),(9,5,'red',10,600,1),(10,1,'red',19,700,1);
/*!40000 ALTER TABLE `tapes` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `user_preferences`
--

DROP TABLE IF EXISTS `user_preferences`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `user_preferences` (
  `id` int(11) NOT NULL AUTO_INCREMENT,
  `username` varchar(100) NOT NULL,
  `language` varchar(10) NOT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=3 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `user_preferences`
--

LOCK TABLES `user_preferences` WRITE;
/*!40000 ALTER TABLE `user_preferences` DISABLE KEYS */;
INSERT INTO `user_preferences` VALUES (1,'admin','en'),(2,'asilva','es');
/*!40000 ALTER TABLE `user_preferences` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `users`
--

DROP TABLE IF EXISTS `users`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `users` (
  `id` int(11) NOT NULL AUTO_INCREMENT,
  `username` varchar(100) NOT NULL,
  `password` varchar(100) NOT NULL,
  `badge_id` varchar(50) NOT NULL,
  `is_admin` tinyint(1) DEFAULT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=3 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `users`
--

LOCK TABLES `users` WRITE;
/*!40000 ALTER TABLE `users` DISABLE KEYS */;
INSERT INTO `users` VALUES (1,'admin','admin','10397',1),(2,'asilva','ISas2903','10395',1);
/*!40000 ALTER TABLE `users` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `work_orders`
--

DROP TABLE IF EXISTS `work_orders`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `work_orders` (
  `id` int(11) NOT NULL AUTO_INCREMENT,
  `part_number` varchar(100) NOT NULL,
  `qty` int(11) NOT NULL,
  `balance` int(11) NOT NULL,
  `type` varchar(20) NOT NULL,
  `created` datetime DEFAULT NULL,
  `started_time` datetime DEFAULT NULL,
  `end_time` datetime DEFAULT NULL,
  `user` varchar(80) NOT NULL,
  `deleted` tinyint(1) DEFAULT NULL,
  `deleted_at` datetime DEFAULT NULL,
  `deleted_user` varchar(80) DEFAULT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=3 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `work_orders`
--

LOCK TABLES `work_orders` WRITE;
/*!40000 ALTER TABLE `work_orders` DISABLE KEYS */;
INSERT INTO `work_orders` VALUES (1,'AN-2025-RND-M',500,0,'kanban','2025-08-12 09:46:43',NULL,NULL,'asilva',0,NULL,NULL),(2,'AN-2025-RND-M',20,0,'kanban','2025-08-12 10:09:31',NULL,NULL,'asilva',0,NULL,NULL);
/*!40000 ALTER TABLE `work_orders` ENABLE KEYS */;
UNLOCK TABLES;
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*!40111 SET SQL_NOTES=@OLD_SQL_NOTES */;

-- Dump completed on 2025-08-12 14:20:26
