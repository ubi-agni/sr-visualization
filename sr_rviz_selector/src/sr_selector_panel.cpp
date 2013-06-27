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

#include "rviz/properties/property_tree_model.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/visualization_manager.h"

#include "sr_selector_panel.h"

namespace sr_rviz_selector
{

SrSelectorPanel::SrSelectorPanel( QWidget* parent )
  : rviz::Panel( parent )
{
  QHBoxLayout* count_layout = new QHBoxLayout;
  count_layout->addWidget( new QLabel( "Selected Count:" ));
  selected_count_editor_ = new QLineEdit;
  count_layout->addWidget( selected_count_editor_ );

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout( count_layout );
  setLayout( layout );

  // Create a timer for polling the selected state.
  // TODO: Should be able to hook a change event somewhere
  // Here we take advantage of QObject's memory management behavior:
  // since "this" is passed to the new QTimer as its parent, the
  // QTimer is deleted by the QObject destructor when this SrSelectorPanel
  // object is destroyed.  Therefore we don't need to keep a pointer
  // to the timer.
  QTimer* output_timer = new QTimer( this );
  connect( output_timer, SIGNAL( timeout() ), this, SLOT( update() ));

  // Next we make signal/slot connections.
  //connect( drive_widget_, SIGNAL( outputVelocity( float, float )), this, SLOT( setVel( float, float )));
  //connect( output_topic_editor_, SIGNAL( editingFinished() ), this, SLOT( updateTopic() ));

  // Start the timer.
  output_timer->start( 100 );
}

void SrSelectorPanel::update()
{
  if( ros::ok() )
  {
    int count = vis_manager_->getSelectionManager()->getPropertyModel()->rowCount();
    std::stringstream ss;
    ss << count;
    selected_count_editor_->setText(QString(ss.str().c_str()));
  }
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void SrSelectorPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  //config.mapSetValue( "Topic", output_topic_ );
}

// Load all configuration data for this panel from the given Config object.
void SrSelectorPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
//  QString topic;
//  if( config.mapGetString( "Topic", &topic ))
//  {
//    output_topic_editor_->setText( topic );
//    updateTopic();
//  }
}

} // end namespace sr_rviz_selector

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sr_rviz_selector::SrSelectorPanel,rviz::Panel )
