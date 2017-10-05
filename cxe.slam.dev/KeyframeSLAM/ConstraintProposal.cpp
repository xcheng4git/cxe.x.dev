#include "stdafx.h"
#include <boost/make_shared.hpp>

#include "ConstraintProposal.h"

namespace slam
{
	namespace constraints
	{
		boost::shared_ptr<ConstraintProposal> ConstraintProposal::createWithIdentity(const slam::KeyframePtr& reference, const slam::KeyframePtr& current)
		{
			boost::shared_ptr<ConstraintProposal> p(new ConstraintProposal());
			p->Reference = reference;
			p->Current = current;
			p->InitialTransformation.setIdentity();

			return p;
		}

		boost::shared_ptr<ConstraintProposal> ConstraintProposal::createWithRelative(const slam::KeyframePtr& reference, const slam::KeyframePtr& current)
		{
			boost::shared_ptr<ConstraintProposal> p(new ConstraintProposal());
			p->Reference = reference;
			p->Current = current;
			p->InitialTransformation = current->pose().inverse() * reference->pose();

			return p;
		}

		ConstraintProposal::ConstraintProposal() :
			InitialTransformation(Eigen::Affine3d::Identity())
		{
		}

		double ConstraintProposal::TotalScore() const
		{
			double s = 0.0;

			for (VoteVector::const_iterator it = Votes.begin(); it != Votes.end(); ++it)
			{
				s += it->Score;
			}

			return s;
		}

		bool ConstraintProposal::Accept() const
		{
			for (VoteVector::const_iterator it = Votes.begin(); it != Votes.end(); ++it)
				if (it->Decision == Vote::Reject) return false;

			return true;
		}

		bool ConstraintProposal::Reject() const
		{
			for (VoteVector::const_iterator it = Votes.begin(); it != Votes.end(); ++it)
				if (it->Decision == Vote::Reject) return true;

			return false;
		}

		void ConstraintProposal::clearVotes()
		{
			Votes.clear();
		}

		boost::shared_ptr<ConstraintProposal> ConstraintProposal::createInverseProposal() const
		{
			boost::shared_ptr<ConstraintProposal> inv(new ConstraintProposal());
			inv->Reference = Current;
			inv->Current = Reference;
			inv->InitialTransformation = InitialTransformation.inverse();

			return inv;
		}

		bool ConstraintProposal::isConstraintBetweenSameFrames(const ConstraintProposal& other)
		{
			return (Reference->id() == other.Reference->id() && Current->id() == other.Current->id()) || (Reference->id() == other.Current->id() && Current->id() == other.Reference->id());
		}

		void ConstraintProposal::printVotingResults(std::ostream& out, const std::string indent) const
		{
			out << indent << "Proposal " << Reference->id() << "->" << Current->id() << " " << (Accept() ? "accept" : "reject") << std::endl;

			for (ConstraintProposal::VoteVector::const_iterator it = Votes.begin(); it != Votes.end(); ++it)
				out << indent << "  " << it->Reason << std::endl;
		}
	}
}