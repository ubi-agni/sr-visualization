/*
 * Copyright 2011 Shadow Robot Company Ltd.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include "sr_selector_panel.h"

namespace sr_rviz_selector
{

// BEGIN_TUTORIAL
// Here is the implementation of the SrSelectorPanel class.  SrSelectorPanel
// has these responsibilities:
//
// - Act as a container for GUI elements DriveWidget and QLineEdit.
// - Publish command velocities 10 times per second (whether 0 or not).
// - Saving and restoring internal state from a config file.
//
// We start with the constructor, doing the standard Qt thing of
// passing the optional *parent* argument on to the superclass
// constructor, and also zero-ing the velocities we will be
// publishing.
SrSelectorPanel::SrSelectorPanel( QWidget* parent )
  : rviz::Panel( parent )
{
  // Next we lay out the "output topic" text entry field using a
  // QLabel and a QLineEdit in a QHBoxLayout.
  QHBoxLayout* topic_layout = new QHBoxLayout;
  topic_layout->addWidget( new QLabel( "Output Topic:" ));
  output_topic_editor_ = new QLineEdit;
  topic_layout->addWidget( output_topic_editor_ );

  // Lay out the topic field above the control widget.
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout( topic_layout );
  setLayout( layout );

  // Create a timer for sending the output.  Motor controllers want to
  // be reassured frequently that they are doing the right thing, so
  // we keep re-sending velocities even when they aren't changing.
  // 
  // Here we take advantage of QObject's memory management behavior:
  // since "this" is passed to the new QTimer as its parent, the
  // QTimer is deleted by the QObject destructor when this SrSelectorPanel
  // object is destroyed.  Therefore we don't need to keep a pointer
  // to the timer.
  QTimer* output_timer = new QTimer( this );

  // Next we make signal/slot connections.
  //connect( drive_widget_, SIGNAL( outputVelocity( float, float )), this, SLOT( setVel( float, float )));
  //connect( output_topic_editor_, SIGNAL( editingFinished() ), this, SLOT( updateTopic() ));
  //connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendVel() ));

  // Start the timer.
  output_timer->start( 100 );

  // Make the control widget start disabled, since we don't start with an output topic.
  //drive_widget_->setEnabled( false );
}

// setVel() is connected to the DriveWidget's output, which is sent
// whenever it changes due to a mouse event.  This just records the
// values it is given.  The data doesn't actually get sent until the
// next timer callback.
void SrSelectorPanel::setVel( float lin, float ang )
{
  //linear_velocity_ = lin;
  //angular_velocity_ = ang;
}

// Read the topic name from the QLineEdit and call setTopic() with the
// results.  This is connected to QLineEdit::editingFinished() which
// fires when the user presses Enter or Tab or otherwise moves focus
// away.
void SrSelectorPanel::updateTopic()
{
  setTopic( output_topic_editor_->text() );
}

// Set the topic name we are publishing to.
void SrSelectorPanel::setTopic( const QString& new_topic )
{
  // Only take action if the name has changed.
  if( new_topic != output_topic_ )
  {
    output_topic_ = new_topic;
    // If the topic is the empty string, don't publish anything.
    //if( output_topic_ == "" )
    //{
    //  velocity_publisher_.shutdown();
    //}
    //else
    //{
    //  // The old ``velocity_publisher_`` is destroyed by this assignment,
    //  // and thus the old topic advertisement is removed.  The call to
    //  // nh_advertise() says we want to publish data on the new topic
    //  // name.
    //  velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>( output_topic_.toStdString(), 1 );
    //}
    //// rviz::Panel defines the configChanged() signal.  Emitting it
    //// tells RViz that something in this panel has changed that will
    //// affect a saved config file.  Ultimately this signal can cause
    //// QWidget::setWindowModified(true) to be called on the top-level
    //// rviz::VisualizationFrame, which causes a little asterisk ("*")
    //// to show in the window's title bar indicating unsaved changes.
    //Q_EMIT configChanged();
  }
}

// Publish the control velocities if ROS is not shutting down and the
// publisher is ready with a valid topic name.
void SrSelectorPanel::sendVel()
{
  if( ros::ok() && velocity_publisher_ )
  {
    //geometry_msgs::Twist msg;
    //msg.linear.x = linear_velocity_;
    //msg.linear.y = 0;
    //msg.linear.z = 0;
    //msg.angular.x = 0;
    //msg.angular.y = 0;
    //msg.angular.z = angular_velocity_;
    //velocity_publisher_.publish( msg );
  }
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void SrSelectorPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  config.mapSetValue( "Topic", output_topic_ );
}

// Load all configuration data for this panel from the given Config object.
void SrSelectorPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString topic;
  if( config.mapGetString( "Topic", &topic ))
  {
    output_topic_editor_->setText( topic );
    updateTopic();
  }
}

} // end namespace sr_rviz_selector

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sr_rviz_selector::SrSelectorPanel,rviz::Panel )
// END_TUTORIAL
